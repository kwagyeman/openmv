# This work is licensed under the MIT license.
# Copyright (c) 2013-2026 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# RTSP H.264 Video + PCM Audio Server
#
# This example streams H.264 video from the hardware encoder AND live
# microphone audio (16-bit PCM) over RTSP at the same time. The microphone
# is captured into a preallocated ring buffer by the audio driver while the
# asyncio server interleaves both streams to the client.
#
# View with e.g.: ffplay -rtsp_transport tcp rtsp://<ip>

import asyncio
import audio
import network
import rtsp_h264
import csi
import time
import codec

AUDIO_RATE = 16000

csi0 = csi.CSI(stream=False)
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.VGA)

# Size the encoder from a snapshot - some sensors letterbox the requested
# framesize (e.g. VGA capture may come back as 640x400).
img = csi0.snapshot()

# Hardware H.264 encoder: one keyframe per second at 30 fps, 1 Mbit/s target.
encoder = codec.H264Encoder(img.width(), img.height(), fps=30, bitrate=1000000, keyframe_interval=30)

# Microphone ring buffer. The audio driver fills it from its callback; the
# server drains it via audio_callback(). Everything is preallocated - no
# allocations happen in the audio path while streaming.
_ring = bytearray(16384)
_ring_mv = memoryview(_ring)
_head = 0
_tail = 0


def _audio_ready(pcm):
    global _head
    n = len(pcm)
    size = len(_ring)
    pos = _head
    first = min(n, size - pos)
    _ring[pos:pos + first] = pcm[:first]
    if first < n:
        _ring[0:n - first] = pcm[first:]
    _head = (pos + n) % size


def audio_callback():
    global _tail
    head = _head
    if _tail == head:
        return None  # nothing pending
    end = head if head > _tail else len(_ring)
    chunk = _ring_mv[_tail:end]
    _tail = end % len(_ring)
    return chunk


audio.init(channels=1, frequency=AUDIO_RATE, gain_db=24)
audio.start_streaming(_audio_ready)

# The microphone's filters settle over the first ~100 ms with a full-scale
# transient that would stream as a loud pop - let it pass and drop it.
time.sleep_ms(300)
_tail = _head

# Setup Network Interface

network_if = network.WLAN(network.STA_IF)
network_if.active(True)
network_if.connect("your-ssid", "your-password")
while not network_if.isconnected():
    print("Trying to connect. Note this may take a while...")
    time.sleep_ms(1000)

# Setup RTSP Server

server = rtsp_h264.rtsp_server(network_if)


def image_callback(pathname, session):
    return csi0.snapshot()


# Stream does not return. The SDP advertises both an H.264 video track and
# an L16 audio track; players that support audio (VLC, ffplay) will play
# both in sync.

try:
    asyncio.run(server.stream_h264(encoder, image_callback,
                                   audio_callback=audio_callback,
                                   audio_rate=AUDIO_RATE))
finally:
    audio.stop_streaming()
