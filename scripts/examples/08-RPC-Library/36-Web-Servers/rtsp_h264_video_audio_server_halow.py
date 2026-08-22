# This work is licensed under the MIT license.
# Copyright (c) 2013-2026 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# RTSP H.264 Video + Audio Server over WiFi HaLow
#
# This example streams H.264 video from the hardware encoder AND the board
# microphone (16-bit PCM) over RTSP, on an 802.11ah (WiFi HaLow) link. The
# server captures the microphone into a preallocated ring buffer and
# interleaves both streams to the client.
#
# HaLow trades bandwidth for range: pick a video bitrate that fits the
# link. An 8MHz channel sustains a few Mbit/s up close; a 1-2MHz channel
# or a long link may only sustain a few hundred kbit/s.
#
# View with e.g.: ffplay -rtsp_transport tcp rtsp://<ip>

import asyncio
import network
import rtsp_h264
import mp4
import audio
import csi
import time
import codec

SSID = "your-ssid"  # 802.11ah access point SSID
KEY = "your-password"  # SAE (WPA3) passphrase, or "" for an open network
COUNTRY = "US"  # Regulatory domain - sets the HaLow channel plan

FRAME_SIZE = csi.VGA  # Drop to csi.QVGA for long links
FPS = 15  # Camera frame rate; the encoder follows real frame times
BITRATE = 500000  # H.264 target bit/s - budget most of the link for video
KEYFRAME_INTERVAL = 15  # One keyframe per second at 15 fps

AUDIO_RATE = 16000  # L16 PCM audio is audio_rate * 16 bit/s on the wire
AUDIO_GAIN_DB = 24

csi0 = csi.CSI(stream=False)
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(FRAME_SIZE)
csi0.framerate(FPS)

W = csi0.width()
H = csi0.height()

encoder = codec.H264Encoder(W, H, fps=FPS, bitrate=BITRATE, keyframe_interval=KEYFRAME_INTERVAL)

# The server drains the microphone ring buffer; the audio driver is
# owned here and just feeds it.
mic = mp4.MicSource(AUDIO_RATE)
audio.init(channels=1, frequency=AUDIO_RATE, gain_db=AUDIO_GAIN_DB)
audio.start_streaming(mic.callback)
mic.settle()  # wait out the mic filters' start-up pop

# Setup Network Interface

network.country(COUNTRY)
network_if = network.HALOW()
network_if.config(pm=network.HALOW.PM_NONE)  # no power saving while streaming
network_if.active(True)
network_if.connect(SSID, KEY)
while not network_if.isconnected():
    print("Trying to connect. Note this may take a while...")
    time.sleep_ms(1000)

# Setup RTSP Server

server = rtsp_h264.rtsp_server(network_if)

_last_rssi_ms = time.ticks_ms()


def image_callback(pathname, session):
    # Report link quality once a second while streaming, for range testing.
    global _last_rssi_ms
    if time.ticks_diff(time.ticks_ms(), _last_rssi_ms) > 1000:
        _last_rssi_ms = time.ticks_ms()
        print("rssi:", network_if.status("rssi"), "dBm")
    return csi0.snapshot()


# Stream does not return. The SDP advertises both an H.264 video track and
# an L16 audio track; players that support audio (VLC, ffplay) will play
# both in sync.

asyncio.run(server.stream_h264(encoder, image_callback,
                               audio_rate=AUDIO_RATE, audio_callback=mic.read))
