/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (C) 2013-2024 OpenMV, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * WINC1500 driver.
 */
#include STM32_HAL_H
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include "py/mphal.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "winc.h"
#include "omv_common.h"

// WINC's includes
#include "driver/include/nmasic.h"
#include "socket/include/socket.h"
#include "programmer/programmer.h"
#include "driver/include/m2m_wifi.h"

static volatile bool ip_obtained = false;
static volatile bool wlan_connected = false;
static volatile uint32_t connected_sta_ip = false;
static volatile bool use_static_ip = false;

static void *async_request_data;
static uint8_t async_request_type = 0;
static volatile bool async_request_done = false;
static volatile bool async_request_ack = false;
static winc_ifconfig_t ifconfig;

typedef struct {
    int size;
    struct sockaddr_in addr;
} recv_from_t;

typedef struct {
    void *arg;
    winc_scan_callback_t cb;
} scan_arg_t;

// Connection state of a socket.
enum {
    WINC_CONN_NONE = 0,
    WINC_CONN_PENDING,
    WINC_CONN_DONE,
    WINC_CONN_FAILED,
};

// A connection accepted by the firmware and waiting for accept() to take it.
typedef struct {
    uint32_t ip;
    uint16_t port;
    int8_t sock;
} winc_accepted_t;

// Per-socket state. Connect, accept and stream receive replies are completed here by
// the event handler whenever it runs, rather than only while a call is waiting on them,
// so a reply that arrives during another call (or while nothing waits, in non-blocking
// mode) is kept instead of dropped, and readiness can be polled.
typedef struct {
    volatile int8_t conn;
    volatile int8_t error;              // Latched socket error, 0 if none.
    volatile bool rx_pending;           // A receive request is outstanding.
    volatile uint8_t accept_count;
    winc_accepted_t accept[WINC_ACCEPT_BACKLOG];
} winc_sock_state_t;

static winc_sock_state_t sock_state[MAX_SOCKET];

// Stream receive buffers, indexed by fd. A receive request stays outstanding across
// calls, so the firmware can write into a buffer at any time until the socket closes.
// They're kept in a root pointer to keep them alive for that long, even if the socket
// object is dropped without being closed.
MP_STATIC_ASSERT(MP_ARRAY_SIZE(MP_STATE_PORT(winc_sockbuf)) == TCP_SOCK_MAX);

static inline bool sock_valid(int fd) {
    return fd >= 0 && fd < MAX_SOCKET;
}

static inline bool sock_is_stream(int fd) {
    return fd >= 0 && fd < TCP_SOCK_MAX;
}

static inline winc_socket_buf_t *sock_rx(int fd) {
    return sock_is_stream(fd) ? MP_STATE_PORT(winc_sockbuf)[fd] : NULL;
}

static void sock_reset(int fd, int8_t conn) {
    if (sock_valid(fd)) {
        memset(&sock_state[fd], 0, sizeof(winc_sock_state_t));
        sock_state[fd].conn = conn;
    }
    if (sock_is_stream(fd)) {
        MP_STATE_PORT(winc_sockbuf)[fd] = NULL;
    }
}

// Queue a connection on its listening socket. The firmware has already accepted it,
// so if the queue is full it's closed rather than leaked.
static void sock_accepted(SOCKET sock, tstrSocketAcceptMsg *msg) {
    if (msg->sock < 0) {
        return;
    }
    winc_sock_state_t *s = &sock_state[sock];
    if (s->accept_count >= WINC_ACCEPT_BACKLOG) {
        WINC1500_EXPORT(close) (msg->sock);
        return;
    }
    winc_accepted_t *a = &s->accept[s->accept_count];
    a->sock = msg->sock;
    a->port = msg->strAddr.sin_port;
    a->ip = msg->strAddr.sin_addr.s_addr;
    s->accept_count++;
}

// Complete a stream receive into the socket's buffer.
static void sock_received(SOCKET sock, tstrSocketRecvMsg *msg) {
    winc_sock_state_t *s = &sock_state[sock];
    winc_socket_buf_t *rx = sock_rx(sock);
    s->rx_pending = false;
    if (rx == NULL) {
        // The socket was closed, nothing to deliver to.
        return;
    }
    if (msg->s16BufferSize > 0) {
        // The data is already in rx->buf, the buffer the request was made with.
        rx->idx = 0;
        rx->size = msg->s16BufferSize;
    } else if (msg->s16BufferSize == SOCK_ERR_NO_ERROR || msg->s16BufferSize == SOCK_ERR_CONN_ABORTED) {
        // The peer closed the connection, latch EOF.
        rx->closed = true;
    } else if (msg->s16BufferSize != SOCK_ERR_TIMEOUT) {
        s->error = msg->s16BufferSize;
    }
}

/**
 * DNS Callback.
 *
 * host: Domain name.
 * ip: Server IP.
 */
static void resolve_callback(uint8_t *host, uint32_t ip) {
    if (async_request_data == NULL) {
        // A reply to a request that has already timed out.
        return;
    }
    async_request_done = true;
    *((uint32_t *) async_request_data) = ip;
}

/**
 * Sockets Callback.
 *
 * sock: Socket descriptor.
 * msg_type: Type of Socket notification. Possible types are:
 *  SOCKET_MSG_BIND
 *  SOCKET_MSG_LISTEN
 *  SOCKET_MSG_ACCEPT
 *  SOCKET_MSG_CONNECT
 *  SOCKET_MSG_SEND
 *  SOCKET_MSG_RECV
 *  SOCKET_MSG_SENDTO
 *  SOCKET_MSG_RECVFROM
 *
 * msg: A structure contains notification informations.
 *  tstrSocketBindMsg
 *  tstrSocketListenMsg
 *  tstrSocketAcceptMsg
 *  tstrSocketConnectMsg
 *  tstrSocketRecvMsg
 */
static void socket_callback(SOCKET sock, uint8_t msg_type, void *msg) {
    // Close Socket on SEND/SENDTO Error.
    // The next SEND/SENDTO operation on the socket will return SOCK_ERR_INVALID_ARG and fail.

    if (((msg_type == SOCKET_MSG_SEND) || (msg_type == SOCKET_MSG_SENDTO))
        && ((*((int16_t *) msg)) < 0)) {
        WINC1500_EXPORT(close) (sock);
        if (sock_valid(sock)) {
            // The socket is gone, so latch an error that can't be mistaken for EAGAIN.
            int16_t error = *((int16_t *) msg);
            sock_state[sock].error = (error == SOCK_ERR_TIMEOUT) ? SOCK_ERR_CONN_ABORTED : error;
        }
    }

    // Replies tracked per socket complete regardless of what (if anything) is waiting.
    if (sock_valid(sock)) {
        switch (msg_type) {
            case SOCKET_MSG_CONNECT: {
                tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *) msg;
                if (sock_state[sock].conn == WINC_CONN_PENDING) {
                    sock_state[sock].conn = (pstrConnect->s8Error == 0) ? WINC_CONN_DONE : WINC_CONN_FAILED;
                }
                return;
            }
            case SOCKET_MSG_ACCEPT:
                sock_accepted(sock, (tstrSocketAcceptMsg *) msg);
                return;
            case SOCKET_MSG_RECV:
                sock_received(sock, (tstrSocketRecvMsg *) msg);
                return;
            default:
                break;
        }
    }

    // Nothing waits on send replies, they only show the firmware is still making progress.
    if (msg_type == SOCKET_MSG_SEND || msg_type == SOCKET_MSG_SENDTO) {
        async_request_ack = true;
        return;
    }

    // A reply to a request that has already timed out has nowhere to go.
    if (async_request_type != msg_type || async_request_data == NULL) {
        debug_printf("spurious message received!"
                     " expected: (%d) received: (%d)\n", async_request_type, msg_type);
        return;
    }

    switch (msg_type) {
        // Socket bind.
        case SOCKET_MSG_BIND: {
            tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *) msg;
            if (pstrBind->status == 0) {
                *((int *) async_request_data) = 0;
                debug_printf("bind success.\n");
            } else {
                *((int *) async_request_data) = -1;
                debug_printf("bind error!\n");
            }
            async_request_done = true;
            break;
        }

        // Socket listen.
        case SOCKET_MSG_LISTEN: {
            tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *) msg;
            if (pstrListen->status == 0) {
                *((int *) async_request_data) = 0;
                debug_printf("listen success.\n");
            } else {
                *((int *) async_request_data) = -1;
                debug_printf("listen error!\n");
            }
            async_request_done = true;
            break;
        }

        case SOCKET_MSG_RECVFROM: {
            tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *) msg;
            recv_from_t *rfrom = (recv_from_t *) async_request_data;

            if (pstrRecv->s16BufferSize > 0) {
                // Get the remote host address and port number
                rfrom->size = pstrRecv->s16BufferSize;
                rfrom->addr.sin_port = pstrRecv->strRemoteAddr.sin_port;
                rfrom->addr.sin_addr = pstrRecv->strRemoteAddr.sin_addr;
                debug_printf("recvfrom size: %d addr:%lu port:%d\n",
                             pstrRecv->s16BufferSize, rfrom->addr.sin_addr.s_addr, rfrom->addr.sin_port);
            } else {
                rfrom->size = pstrRecv->s16BufferSize;
                rfrom->addr.sin_port = 0;
                rfrom->addr.sin_addr.s_addr = 0;
                debug_printf("recvfrom error:%d\n", pstrRecv->s16BufferSize);
            }
            async_request_done = true;
            break;
        }

        default:
            debug_printf("Unknown message type: %d\n", msg_type);
            break;
    }
}

/**
 * WiFi Callback in AP mode.
 *
 * msg_type: type of Wi-Fi notification. Possible types are:
 *  M2M_WIFI_RESP_CON_STATE_CHANGED
 *  M2M_WIFI_RESP_CONN_INFO
 *  M2M_WIFI_REQ_DHCP_CONF
 *  M2M_WIFI_REQ_WPS
 *  M2M_WIFI_RESP_IP_CONFLICT
 *  M2M_WIFI_RESP_SCAN_DONE
 *  M2M_WIFI_RESP_SCAN_RESULT
 *  M2M_WIFI_RESP_CURRENT_RSSI
 *  M2M_WIFI_RESP_CLIENT_INFO
 *  M2M_WIFI_RESP_PROVISION_INFO
 *  M2M_WIFI_RESP_DEFAULT_CONNECT
 *
 * In case Bypass mode is defined :
 *      M2M_WIFI_RESP_ETHERNET_RX_PACKET
 *
 * In case Monitoring mode is used:
 *      M2M_WIFI_RESP_WIFI_RX_PACKET
 *
 * msg: A pointer to a buffer containing the notification parameters (if any).
 * It should be casted to the correct data type corresponding to the notification type.
 */

static void wifi_callback_ap(uint8_t msg_type, void *msg) {
    switch (msg_type) {
        case M2M_WIFI_RESP_CON_STATE_CHANGED: {
            tstrM2mWifiStateChanged *wifi_state = (tstrM2mWifiStateChanged *) msg;
            if (wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
                debug_printf("Station connected\r\n");
            } else if (wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
                connected_sta_ip = 0;
                debug_printf("Station disconnected\r\n");
            }
            break;
        }

        case M2M_WIFI_REQ_DHCP_CONF: {
            uint8_t *ip = (uint8_t *) msg;
            debug_printf("Station connected. IP is %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
            ((uint8_t *) &connected_sta_ip)[0] = ip[0];
            ((uint8_t *) &connected_sta_ip)[1] = ip[1];
            ((uint8_t *) &connected_sta_ip)[2] = ip[2];
            ((uint8_t *) &connected_sta_ip)[3] = ip[3];
            break;
        }

        default:
            debug_printf("Unknown message type: %d\n", msg_type);
            break;
    }
}

/**
 * WiFi Callback in STA mode.
 *
 * msg_type: type of Wi-Fi notification. Possible types are:
 *  M2M_WIFI_RESP_CON_STATE_CHANGED
 *  M2M_WIFI_RESP_CONN_INFO
 *  M2M_WIFI_REQ_DHCP_CONF
 *  M2M_WIFI_REQ_WPS
 *  M2M_WIFI_RESP_IP_CONFLICT
 *  M2M_WIFI_RESP_SCAN_DONE
 *  M2M_WIFI_RESP_SCAN_RESULT
 *  M2M_WIFI_RESP_CURRENT_RSSI
 *  M2M_WIFI_RESP_CLIENT_INFO
 *  M2M_WIFI_RESP_PROVISION_INFO
 *  M2M_WIFI_RESP_DEFAULT_CONNECT
 *
 * In case Bypass mode is defined :
 *      M2M_WIFI_RESP_ETHERNET_RX_PACKET
 *
 * In case Monitoring mode is used:
 *      M2M_WIFI_RESP_WIFI_RX_PACKET
 *
 * msg: A pointer to a buffer containing the notification parameters (if any).
 * It should be casted to the correct data type corresponding to the notification type.
 */
static void wifi_callback_sta(uint8_t msg_type, void *msg) {
    // Index of scan list to request scan result.
    static uint8_t scan_request_index = 0;

    switch (msg_type) {

        case M2M_WIFI_RESP_CURRENT_RSSI: {
            if (async_request_data == NULL) {
                // A reply to a request that has already timed out.
                break;
            }
            int rssi = *((int8_t *) msg);
            *((int *) async_request_data) = rssi;
            async_request_done = true;
            break;
        }

        case M2M_WIFI_RESP_CON_STATE_CHANGED: {
            tstrM2mWifiStateChanged *wifi_state = (tstrM2mWifiStateChanged *) msg;
            if (wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
                wlan_connected = true;
                if (use_static_ip) {
                    tstrM2MIPConfig ipconfig;
                    ipconfig.u32StaticIP = *((uint32_t *) ifconfig.ip_addr);
                    ipconfig.u32SubnetMask = *((uint32_t *) ifconfig.subnet_addr);
                    ipconfig.u32Gateway = *((uint32_t *) ifconfig.gateway_addr);
                    ipconfig.u32DNS = *((uint32_t *) ifconfig.dns_addr);
                    m2m_wifi_set_static_ip(&ipconfig);
                    // NOTE: the async request is done because there's
                    // no DHCP request/response when using a static IP.
                    // Set the ip_obtained flag to true anyway...
                    ip_obtained = true;
                    async_request_done = true;
                }
            } else if (wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
                ip_obtained = false;
                wlan_connected = false;
                async_request_done = true;
            }
            break;
        }

        case M2M_WIFI_REQ_DHCP_CONF: {
            ip_obtained = true;
            async_request_done = true;
            tstrM2MIPConfig *ipconfig = (tstrM2MIPConfig *) msg;
            memcpy(ifconfig.ip_addr, &ipconfig->u32StaticIP, WINC_IPV4_ADDR_LEN);
            memcpy(ifconfig.subnet_addr, &ipconfig->u32SubnetMask, WINC_IPV4_ADDR_LEN);
            memcpy(ifconfig.gateway_addr, &ipconfig->u32Gateway, WINC_IPV4_ADDR_LEN);
            memcpy(ifconfig.dns_addr, &ipconfig->u32DNS, WINC_IPV4_ADDR_LEN);
            break;
        }

        case M2M_WIFI_RESP_CONN_INFO: {
            if (async_request_data == NULL) {
                // A reply to a request that has already timed out.
                break;
            }
            // Connection info
            tstrM2MConnInfo *con_info = (tstrM2MConnInfo *) msg;
            winc_netinfo_t *netinfo = (winc_netinfo_t *) async_request_data;

            // Set rssi and security
            netinfo->rssi = con_info->s8RSSI;
            netinfo->security = con_info->u8SecType;

            // Copy IP address.
            memcpy(netinfo->ip_addr, con_info->au8IPAddr, WINC_IPV4_ADDR_LEN);

            // Get MAC Address.
            memcpy(netinfo->mac_addr, con_info->au8MACAddress, WINC_MAC_ADDR_LEN);

            // Copy SSID.
            con_info->acSSID[WINC_MAX_SSID_LEN - 1] = 0;
            strncpy(netinfo->ssid, con_info->acSSID, WINC_MAX_SSID_LEN);

            async_request_done = true;
            break;
        }

        case M2M_WIFI_RESP_SCAN_DONE: {
            if (async_request_data == NULL) {
                // A reply to a request that has already timed out.
                break;
            }
            scan_request_index = 0;
            tstrM2mScanDone *scan_result = (tstrM2mScanDone *) msg;

            // The number of APs found in the last scan request.
            if (scan_result->u8NumofCh <= 0) {
                // Nothing found.
                async_request_done = true;
            } else {
                // Found APs, request scan results.
                m2m_wifi_req_scan_result(scan_request_index++);
            }

            break;
        }

        case M2M_WIFI_RESP_SCAN_RESULT: {
            if (async_request_data == NULL) {
                // A reply to a request that has already timed out.
                break;
            }
            tstrM2mWifiscanResult *scan_result;
            scan_result = (tstrM2mWifiscanResult *) msg;

            winc_scan_result_t wscan_result;
            scan_arg_t *scan_arg = (scan_arg_t *) async_request_data;

            // Set channel, rssi and security.
            wscan_result.channel = scan_result->u8ch;
            wscan_result.rssi = scan_result->s8rssi;
            wscan_result.security = scan_result->u8AuthType;

            // Copy BSSID and SSID.
            memcpy(wscan_result.bssid, scan_result->au8BSSID, WINC_MAC_ADDR_LEN);
            scan_result->au8SSID[WINC_MAX_SSID_LEN - 1] = 0;
            strncpy((char *) wscan_result.ssid, (const char *) scan_result->au8SSID, WINC_MAX_SSID_LEN);

            // Call scan result callback
            scan_arg->cb(&wscan_result, scan_arg->arg);

            int num_found_ap = m2m_wifi_get_num_ap_found();
            if (num_found_ap == scan_request_index) {
                async_request_done = true;
            } else {
                // Request next scan result
                m2m_wifi_req_scan_result(scan_request_index++);
            }
            break;
        }

        default:
            debug_printf("Unknown message type: %d\n", msg_type);
            break;
    }
}

static int winc_async_request(uint8_t msg_type, void *ret, uint32_t timeout) {
    // Do async request.
    async_request_data = ret;
    async_request_ack = false;
    async_request_done = false;
    async_request_type = msg_type;
    uint32_t tick_start = HAL_GetTick();

    if (timeout == 0) {
        // In non-blocking mode, set timeout to a small timeout other
        // than zero, to allow the socket function to time out first.
        timeout = 10;
    }

    // Wait for async request to finish.
    while (async_request_done == false) {
        // Handle pending events from network controller.
        m2m_wifi_handle_events(NULL);

        if (async_request_ack == true) {
            // Received an ACK event for an older send/sendto() instead
            // of the expected event. Reset the timeout and keep trying.
            async_request_ack = false;
            tick_start = HAL_GetTick();
        }

        if ((HAL_GetTick() - tick_start) >= timeout) {
            break;
        }

        // Wait for the next IRQ.
        __WFI();
    }

    // The request buffer is owned by the caller's stack frame. Drop it, so that a
    // reply arriving after a timeout isn't written to a frame that no longer exists.
    async_request_data = NULL;
    return async_request_done ? SOCK_ERR_NO_ERROR : SOCK_ERR_TIMEOUT;
}

const char *winc_strerror(int error) {
    static const char *winc_errors[] = {
        "Success.",
        "Error sending packet!",
        "Error receiving packet!",
        "Memory alloc failed!",
        "Timeout!",
        "Init failed!",
        "Bus error!",
        "Not ready!",
        "Firmware error!",
        "SPI bus error!",
        "Burn firmware failed!",
        "Ack.",
        "Failed!.",
        "Firmware version mismatch. Please update WINC1500 firmware, see Examples->14-WiFi-Shield->fw_update.py",
        "Scan in progress.",
        "Invalid Arg",
    };

    error = -error;
    if (error > (sizeof(winc_errors) / sizeof(winc_errors[0]))) {
        return "unknown error";
    } else {
        return winc_errors[error];
    }
}

int winc_init(winc_mode_t winc_mode) {
    int error = 0;

    // Initialize the BSP.
    nm_bsp_init();

    // Use DHCP by default.
    use_static_ip = false;

    // Reset ifconfig info.
    memset(&ifconfig, 0, sizeof(winc_ifconfig_t));

    switch (winc_mode) {
        case WINC_MODE_BSP: {
            // Initialize the BSP and return.
            m2m_wifi_init_hold();
            break;
        }

        case WINC_MODE_FIRMWARE: {
            // Enter download mode.
            error = m2m_wifi_download_mode();
            if (error != M2M_SUCCESS) {
                return error;
            }
            break;
        }

        case WINC_MODE_AP:
        case WINC_MODE_STA: {
            tstrWifiInitParam param;
            // Initialize Wi-Fi parameters structure.
            memset((uint8_t *) &param, 0, sizeof(tstrWifiInitParam));
            if (winc_mode == WINC_MODE_AP) {
                param.pfAppWifiCb = wifi_callback_ap;
            } else {
                param.pfAppWifiCb = wifi_callback_sta;
            }

            // Initialize Wi-Fi driver with data and status callbacks.
            error = m2m_wifi_init(&param);
            if (error != M2M_SUCCESS) {
                return error;
            }

            uint8_t mac_addr_valid;
            uint8_t mac_addr[M2M_MAC_ADDRES_LEN];
            // Get MAC Address from OTP.
            m2m_wifi_get_otp_mac_address(mac_addr, &mac_addr_valid);

            if (!mac_addr_valid) {
                // User define MAC Address.
                const char main_user_define_mac_address[] = {0xf8, 0xf0, 0x05, 0x20, 0x0b, 0x09};
                // Cannot found MAC Address from OTP. Set user define MAC address.
                m2m_wifi_set_mac_address((uint8_t *) main_user_define_mac_address);
            }

            // Initialize socket layer.
            socketDeinit();
            socketInit();
            for (int fd = 0; fd < MAX_SOCKET; fd++) {
                sock_reset(fd, WINC_CONN_NONE);
            }

            // Register sockets callback functions
            registerSocketCallback(socket_callback, resolve_callback);
            break;
        }
        default:
            break;
    }

    return 0;
}

int winc_connect(const char *ssid, uint8_t security, const char *key, uint16_t channel) {
    async_request_data = &ifconfig;

    // Drain events left over from a previous connection, such as the state change
    // that disconnect() requests but never waits for, so that they can't satisfy
    // the wait below before this connection has even been attempted.
    m2m_wifi_handle_events(NULL);
    ip_obtained = false;
    wlan_connected = false;

    //Disable/Enable DHCP client before connecting.
    m2m_wifi_enable_dhcp(!use_static_ip);

    // Connect to AP
    if (m2m_wifi_connect((char *) ssid, strlen(ssid), security, (void *) key, M2M_WIFI_CH_ALL) != 0) {
        return -1;
    }

    // Wait for the connection to come all the way up. Note this waits on the
    // connection state, not on any event arriving: a failed connection reports
    // the same state change as a disconnection, and would otherwise be reported
    // to the caller as success.
    mp_uint_t tick_start = mp_hal_ticks_ms();
    while (winc_isconnected() == 0) {
        // Handle pending events from network controller.
        m2m_wifi_handle_events(NULL);
        // Wait on MicroPython's event handler rather than WFI, so that scheduled
        // callbacks still run and a KeyboardInterrupt can break out of the wait.
        mp_event_wait_ms(1);
        if ((mp_hal_ticks_ms() - tick_start) >= WINC_CONNECT_TIMEOUT) {
            return -1;
        }
    }
    return 0;
}

int winc_start_ap(const char *ssid, uint8_t security, const char *key, uint16_t channel) {
    tstrM2MAPConfig apconfig;

    memset(&apconfig, 0, sizeof(tstrM2MAPConfig));
    strcpy((char *) &apconfig.au8SSID, ssid);
    apconfig.u8ListenChannel = channel;
    apconfig.u8SecType = security;
    apconfig.u8SsidHide = false;

    apconfig.au8DHCPServerIP[0] = 192;
    apconfig.au8DHCPServerIP[1] = 168;
    apconfig.au8DHCPServerIP[2] = 1;
    apconfig.au8DHCPServerIP[3] = 1;

    memcpy(ifconfig.ip_addr, apconfig.au8DHCPServerIP, WINC_IPV4_ADDR_LEN);
    memcpy(ifconfig.subnet_addr, "\xff\xff\xff\x00", WINC_IPV4_ADDR_LEN);
    memcpy(ifconfig.gateway_addr, apconfig.au8DHCPServerIP, WINC_IPV4_ADDR_LEN);
    memcpy(ifconfig.dns_addr, apconfig.au8DHCPServerIP, WINC_IPV4_ADDR_LEN);

    if (security != M2M_WIFI_SEC_OPEN) {
        size_t size = OMV_MIN(strlen(key), M2M_MAX_PSK_LEN);
        memcpy(&apconfig.au8Key, key, size);
        apconfig.u8KeySz = size;
    }

    // Initialize WiFi in AP mode.
    if (m2m_wifi_enable_ap(&apconfig) != M2M_SUCCESS) {
        return -1;
    }

    return 0;
}

int winc_disconnect() {
    int ret = m2m_wifi_disconnect();
    // Let the state change land here rather than leaving it pending for the
    // next request to trip over.
    m2m_wifi_handle_events(NULL);
    ip_obtained = false;
    wlan_connected = false;
    return ret;
}

int winc_isconnected() {
    return (wlan_connected && ip_obtained);
}

int winc_connected_sta(uint32_t *sta_ip) {
    if (connected_sta_ip == 0) {
        m2m_wifi_handle_events(NULL);
    }

    *sta_ip = connected_sta_ip;
    return 0;
}

int winc_wait_for_sta(uint32_t *sta_ip, uint32_t timeout) {
    uint32_t tick_start = HAL_GetTick();
    while (connected_sta_ip == 0) {
        __WFI();
        // Handle pending events from network controller.
        m2m_wifi_handle_events(NULL);
        if ((HAL_GetTick() - tick_start) >= timeout) {
            break;
        }
    }

    *sta_ip = connected_sta_ip;
    return 0;
}

int winc_ifconfig(winc_ifconfig_t *rifconfig, bool set) {
    if (set) {
        use_static_ip = true;
        memcpy(&ifconfig, rifconfig, sizeof(winc_ifconfig_t));
    } else {
        // Copy the ifconfig info stored from the DHCP request.
        memcpy(rifconfig->ip_addr, ifconfig.ip_addr, WINC_IPV4_ADDR_LEN);
        memcpy(rifconfig->subnet_addr, ifconfig.subnet_addr, WINC_IPV4_ADDR_LEN);
        memcpy(rifconfig->gateway_addr, ifconfig.gateway_addr, WINC_IPV4_ADDR_LEN);
        memcpy(rifconfig->dns_addr, ifconfig.dns_addr, WINC_IPV4_ADDR_LEN);
    }
    return 0;
}

// Wait for an async request to complete. The request buffer is owned by the
// caller's stack frame, so it's dropped on the way out: without that, a reply
// arriving after a timeout would be written to a frame that no longer exists.
static int winc_wait_for_request(uint32_t timeout) {
    int ret = 0;
    mp_uint_t tick_start = mp_hal_ticks_ms();

    while (async_request_done == false) {
        // Handle pending events from network controller.
        m2m_wifi_handle_events(NULL);
        mp_event_wait_ms(1);
        if ((mp_hal_ticks_ms() - tick_start) >= timeout) {
            ret = -1;
            break;
        }
    }

    async_request_data = NULL;
    return ret;
}

int winc_netinfo(winc_netinfo_t *netinfo) {
    async_request_done = false;
    async_request_data = netinfo;

    // Request connection info
    m2m_wifi_get_connection_info();

    return winc_wait_for_request(WINC_REQUEST_TIMEOUT);
}

int winc_scan(winc_scan_callback_t cb, void *arg) {
    scan_arg_t scan_arg = {arg, cb};
    async_request_done = false;
    async_request_data = &scan_arg;

    // Request scan.
    m2m_wifi_request_scan(M2M_WIFI_CH_ALL);

    return winc_wait_for_request(WINC_SCAN_TIMEOUT);
}

int winc_get_rssi(int *rssi) {
    async_request_done = false;
    async_request_data = rssi;

    // Request RSSI.
    m2m_wifi_req_curr_rssi();

    return winc_wait_for_request(WINC_REQUEST_TIMEOUT);
}

int winc_fw_version(winc_fwver_t *wfwver) {
    tstrM2mRev fwver;

    // Read FW, Driver and HW versions.
    m2m_wifi_get_firmware_version(&fwver);

    wfwver->fw_major = fwver.u8FirmwareMajor;               // Firmware version major number.
    wfwver->fw_minor = fwver.u8FirmwareMinor;               // Firmware version minor number.
    wfwver->fw_patch = fwver.u8FirmwarePatch;               // Firmware version patch number.
    wfwver->drv_major = M2M_RELEASE_VERSION_MAJOR_NO;       // Driver version major number.
    wfwver->drv_minor = M2M_RELEASE_VERSION_MINOR_NO;       // Driver version minor number.
    wfwver->drv_patch = M2M_RELEASE_VERSION_PATCH_NO;       // Driver version patch number.
    wfwver->chip_id = fwver.u32Chipid;                      // HW revision number (chip ID).
    return 0;
}

int winc_flash_dump(const char *path) {
    if (dump_firmware(path) != M2M_SUCCESS) {
        return -1;
    }
    return 0;
}

int winc_flash_erase() {
    // Erase the WINC1500 flash.
    if (programmer_erase_all() != M2M_SUCCESS) {
        return -1;
    }
    return 0;
}

int winc_flash_write(const char *path) {
    // Program the firmware on the WINC1500 flash.
    if (burn_firmware(path) != M2M_SUCCESS) {
        return -1;
    }
    return 0;
}

int winc_flash_verify(const char *path) {
    // Verify the firmware on the WINC1500 flash.
    if (verify_firmware(path) != M2M_SUCCESS) {
        return -1;
    }
    return 0;
}

int winc_gethostbyname(const char *name, uint8_t *out_ip) {
    int ret;
    uint32_t ip = 0;
    ret = WINC1500_EXPORT(gethostbyname) ((uint8_t *) name);
    if (ret == SOCK_ERR_NO_ERROR) {
        ret = winc_async_request(0, &ip, WINC_REQUEST_TIMEOUT);
    } else {
        return -1;
    }

    if (ip == 0) {
        // unknown host
        return ENOENT;
    }

    out_ip[0] = ip;
    out_ip[1] = ip >> 8;
    out_ip[2] = ip >> 16;
    out_ip[3] = ip >> 24;
    return 0;
}

int winc_socket_socket(uint8_t type) {
    // open socket
    int fd = WINC1500_EXPORT(socket) (AF_INET, type, 0);
    if (fd >= 0) {
        sock_reset(fd, WINC_CONN_NONE);
    }
    return fd;
}

void winc_socket_set_buf(int fd, winc_socket_buf_t *sockbuf) {
    if (sock_is_stream(fd)) {
        MP_STATE_PORT(winc_sockbuf)[fd] = sockbuf;
    }
}

void winc_socket_close(int fd) {
    // Close connections the firmware accepted on a listening socket but nothing took.
    if (sock_valid(fd)) {
        for (int i = 0; i < sock_state[fd].accept_count; i++) {
            WINC1500_EXPORT(close) (sock_state[fd].accept[i].sock);
        }
    }
    WINC1500_EXPORT(close) (fd);
    // Drops the receive buffer: a reply to a request still outstanding is discarded by the
    // socket layer, since closing the socket ended its session.
    sock_reset(fd, WINC_CONN_NONE);
}

// Wait for a socket event: returns false once the timeout has expired, otherwise waits
// for the next interrupt and handles pending events. Waiting on MicroPython's event
// handler rather than WFI lets a KeyboardInterrupt break out of a blocking call. With
// a zero timeout this returns false straight away, for non-blocking calls.
static bool winc_socket_wait(mp_uint_t tick_start, uint32_t timeout) {
    if ((mp_hal_ticks_ms() - tick_start) >= timeout) {
        return false;
    }
    mp_event_wait_ms(1);
    m2m_wifi_handle_events(NULL);
    return true;
}

// Keep a receive request outstanding on a connected stream socket whose buffer is
// empty, so that data (or the peer closing) arrives without a blocking call.
static int winc_socket_recv_start(int fd) {
    winc_sock_state_t *s = &sock_state[fd];
    winc_socket_buf_t *rx = sock_rx(fd);
    if (rx == NULL || s->conn != WINC_CONN_DONE || s->rx_pending ||
        s->error || rx->size || rx->closed) {
        return SOCK_ERR_NO_ERROR;
    }
    rx->idx = 0;
    // Zero timeout means the firmware waits forever: the request stays outstanding
    // until data arrives, and the host side applies the socket's timeout.
    int ret = WINC1500_EXPORT(recv) (fd, rx->buf, WINC_SOCKBUF_MAX_SIZE, 0);
    if (ret == SOCK_ERR_NO_ERROR) {
        s->rx_pending = true;
    }
    return ret;
}

int winc_socket_poll(int fd) {
    if (!sock_valid(fd)) {
        return WINC_POLL_NVAL;
    }

    winc_sock_state_t *s = &sock_state[fd];
    winc_socket_buf_t *rx = sock_rx(fd);

    // Handle pending events, then keep a receive outstanding for the next poll.
    m2m_wifi_handle_events(NULL);
    winc_socket_recv_start(fd);

    int ret = 0;
    if (!sock_is_stream(fd)) {
        // Datagrams are received with blocking requests, so only sending is polled.
        ret |= WINC_POLL_WR;
    } else if (s->conn == WINC_CONN_FAILED || s->error) {
        ret |= WINC_POLL_ERR;
    } else {
        if (s->accept_count || (rx != NULL && (rx->size || rx->closed))) {
            ret |= WINC_POLL_RD;
        }
        if (s->conn == WINC_CONN_DONE) {
            // The firmware doesn't report free transmit space, a send that finds the
            // buffers full fails with EAGAIN.
            ret |= WINC_POLL_WR;
        }
    }
    return ret;
}

// Check that stream I/O can proceed on a socket.
static int winc_socket_check_conn(int fd) {
    if (!sock_valid(fd)) {
        return SOCK_ERR_INVALID_ARG;
    }
    winc_sock_state_t *s = &sock_state[fd];
    if (s->error) {
        return s->error;
    }
    switch (s->conn) {
        case WINC_CONN_DONE:
            return SOCK_ERR_NO_ERROR;
        case WINC_CONN_PENDING:
            return SOCK_ERR_TIMEOUT;
        case WINC_CONN_FAILED:
            return WINC_SOCK_ERR_CONN_FAILED;
        default:
            // Datagram sockets don't need to connect.
            return sock_is_stream(fd) ? WINC_SOCK_ERR_NOT_CONNECTED : SOCK_ERR_NO_ERROR;
    }
}

int winc_socket_bind(int fd, sockaddr *addr) {
    // Call bind and check HIF errors.
    int ret = WINC1500_EXPORT(bind) (fd, addr, sizeof(*addr));
    if (ret == SOCK_ERR_NO_ERROR) {
        // Do async request
        ret = winc_async_request(SOCKET_MSG_BIND, &ret, WINC_REQUEST_TIMEOUT);
    }

    return ret;
}

int winc_socket_listen(int fd, uint32_t backlog) {
    // Call listen and check HIF errors.
    int ret = WINC1500_EXPORT(listen) (fd, backlog);
    if (ret == SOCK_ERR_NO_ERROR) {
        // Do async request
        ret = winc_async_request(SOCKET_MSG_LISTEN, &ret, WINC_REQUEST_TIMEOUT);
    }

    return ret;
}

int winc_socket_accept(int fd, sockaddr *addr, int *fd_out, uint32_t timeout) {
    if (!sock_valid(fd)) {
        return SOCK_ERR_INVALID_ARG;
    }

    // Call accept and check HIF errors.
    int ret = WINC1500_EXPORT(accept) (fd, NULL, 0);
    if (ret != SOCK_ERR_NO_ERROR) {
        return ret;
    }

    // The firmware accepts connections by itself, wait for one to be queued.
    winc_sock_state_t *s = &sock_state[fd];
    mp_uint_t tick_start = mp_hal_ticks_ms();
    m2m_wifi_handle_events(NULL);
    while (s->accept_count == 0) {
        if (!winc_socket_wait(tick_start, timeout)) {
            return SOCK_ERR_TIMEOUT;
        }
    }

    winc_accepted_t a = s->accept[0];
    s->accept_count--;
    memmove(&s->accept[0], &s->accept[1], s->accept_count * sizeof(winc_accepted_t));

    sockaddr_in *addr_in = (sockaddr_in *) addr;
    memset(addr, 0, sizeof(*addr));
    addr_in->sin_family = AF_INET;
    addr_in->sin_port = a.port;
    addr_in->sin_addr.s_addr = a.ip;
    *fd_out = a.sock;
    sock_reset(a.sock, WINC_CONN_DONE);
    return SOCK_ERR_NO_ERROR;
}

int winc_socket_connect(int fd, sockaddr *addr, uint32_t timeout) {
    if (!sock_valid(fd)) {
        return SOCK_ERR_INVALID_ARG;
    }

    winc_sock_state_t *s = &sock_state[fd];
    if (s->conn == WINC_CONN_NONE) {
        int ret = WINC1500_EXPORT(connect) (fd, addr, sizeof(*addr));
        if (ret != SOCK_ERR_NO_ERROR) {
            return ret;
        }
        s->conn = WINC_CONN_PENDING;
    }

    mp_uint_t tick_start = mp_hal_ticks_ms();
    m2m_wifi_handle_events(NULL);
    while (s->conn == WINC_CONN_PENDING) {
        if (!winc_socket_wait(tick_start, timeout)) {
            // In non-blocking mode the connection completes in the background.
            return (timeout == 0) ? WINC_SOCK_ERR_INPROGRESS : SOCK_ERR_TIMEOUT;
        }
    }

    return (s->conn == WINC_CONN_DONE) ? SOCK_ERR_NO_ERROR : WINC_SOCK_ERR_CONN_FAILED;
}

int winc_socket_send(int fd, const uint8_t *buf, uint32_t len, uint32_t timeout) {
    uint32_t tick_start = HAL_GetTick();

    int bytes = 0;

    int ret = winc_socket_check_conn(fd);
    if (ret != SOCK_ERR_NO_ERROR) {
        return ret;
    }

    while (bytes < len) {
        // Handle pending events, send replies free transmit buffers.
        m2m_wifi_handle_events(NULL);

        // Split the packet into smaller ones.
        int n = OMV_MIN((len - bytes), SOCKET_BUFFER_MAX_LENGTH);
        ret = WINC1500_EXPORT(send) (fd, (uint8_t *) buf + bytes, n, 0);

        if (ret == SOCK_ERR_NO_ERROR) {
            bytes += n;
        } else if (ret == SOCK_ERR_BUFFER_FULL) {
            if ((HAL_GetTick() - tick_start) >= timeout) {
                break;
            }
        } else {
            // another error
            return ret;
        }
    }

    // Nothing could be sent before the timeout (EAGAIN in non-blocking mode).
    return (bytes || len == 0) ? bytes : SOCK_ERR_TIMEOUT;
}

int winc_socket_recv(int fd, uint8_t *buf, uint32_t len, uint32_t timeout) {
    mp_uint_t tick_start = mp_hal_ticks_ms();
    m2m_wifi_handle_events(NULL);

    for (;;) {
        // Re-read each time, a callback run while waiting could close the socket.
        winc_socket_buf_t *rx = sock_rx(fd);
        if (rx == NULL) {
            return SOCK_ERR_INVALID_ARG;
        }

        if (rx->size) {
            uint32_t bytes = OMV_MIN(len, rx->size);
            memcpy(buf, rx->buf + rx->idx, bytes);
            rx->idx += bytes;
            rx->size -= bytes;
            if (rx->size == 0) {
                // Request more data while the caller is busy with this.
                winc_socket_recv_start(fd);
            }
            return bytes;
        }

        if (rx->closed) {
            // The peer has closed the connection, return EOF without another HIF request.
            return 0;
        }

        int ret = winc_socket_check_conn(fd);
        if (ret == SOCK_ERR_NO_ERROR) {
            ret = winc_socket_recv_start(fd);
        }

        // Wait for data, or for the connection to complete. A full HIF is retried.
        if (ret != SOCK_ERR_NO_ERROR && ret != SOCK_ERR_TIMEOUT && ret != SOCK_ERR_BUFFER_FULL) {
            return ret;
        }

        if (!winc_socket_wait(tick_start, timeout)) {
            return SOCK_ERR_TIMEOUT;
        }
    }
}

int winc_socket_sendto(int fd, const uint8_t *buf, uint32_t len, sockaddr *addr, uint32_t timeout) {
    uint32_t tick_start = HAL_GetTick();

    int bytes = 0;

    while (bytes < len) {
        // Handle pending events, send replies free transmit buffers.
        m2m_wifi_handle_events(NULL);

        // Split the packet into smaller ones.
        int n = OMV_MIN((len - bytes), SOCKET_BUFFER_MAX_LENGTH);
        int ret = WINC1500_EXPORT(sendto) (fd, (uint8_t *) buf + bytes, n, 0, addr, sizeof(*addr));

        if (ret == SOCK_ERR_NO_ERROR) {
            bytes += n;
        } else if (ret == SOCK_ERR_BUFFER_FULL) {
            if ((HAL_GetTick() - tick_start) >= timeout) {
                break;
            }
        } else {
            // another error
            return ret;
        }
    }

    return bytes;
}

int winc_socket_recvfrom(int fd, uint8_t *buf, uint32_t len, sockaddr *addr, uint32_t timeout) {
    memset(addr, 0, sizeof(sockaddr));

    // The firmware never replies to requests larger than the datagram it can deliver.
    len = OMV_MIN(len, WINC_MAX_DGRAM_SIZE);

    // A zero timeout makes the firmware wait forever, and a reply after the host stopped
    // waiting would land in a buffer the caller no longer owns. Use the shortest timeout.
    recv_from_t rfrom;
    int ret = WINC1500_EXPORT(recvfrom) (fd, buf, len, timeout ? timeout : 1);
    if (ret == SOCK_ERR_NO_ERROR) {
        // Do async request
        // Note: Double timeout to ensure the socket function times out first.
        ret = winc_async_request(SOCKET_MSG_RECVFROM, &rfrom, timeout * 2);
    }

    // Check received bytes returned from async request.
    if (ret != SOCK_ERR_NO_ERROR || rfrom.size <= 0) {
        return (ret != SOCK_ERR_NO_ERROR) ? ret : rfrom.size;
    }

    *addr = *((struct sockaddr *) &rfrom.addr);
    return rfrom.size;
}

int winc_socket_setsockopt(int fd, uint32_t level, uint32_t opt, const void *optval, uint32_t optlen) {
    return WINC1500_EXPORT(setsockopt) (fd, level, opt, optval, optlen);
}
