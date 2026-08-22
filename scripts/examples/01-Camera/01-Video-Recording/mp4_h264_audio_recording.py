# This work is licensed under the MIT license.
# Copyright (c) 2013-2026 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# H.264 MP4 Video + PCM Audio Recording
#
# Note: You will need an SD card to run this example.
#
# Records H.264 video from the hardware encoder AND live microphone audio
# (16-bit PCM) into one standard MP4 file. PCM audio plays in VLC/ffmpeg/
# QuickTime but not in the built-in Windows players or browsers - remux on
# the desktop for those: ffmpeg -i in.mp4 -c:v copy -c:a aac out.mp4 The microphone is captured into
# a preallocated ring buffer by the audio driver while the main loop drains
# it into the muxer alongside each video frame.

import audio
import csi
import codec
import mp4
import time
import machine
import struct

AUDIO_RATE = 16000
FPS = 30  # The camera frame rate drives the recording cadence; the encoder
# follows the real frame times passed via timestamp_us.
RECORD_TIME = 10  # seconds

csi0 = csi.CSI(stream=False)
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.VGA)
csi0.framerate(FPS)

# Size the encoder from a snapshot - some sensors letterbox the requested
# framesize (e.g. VGA capture may come back as 640x400).
img = csi0.snapshot()

# Hardware H.264 encoder: one keyframe per second, 1 Mbit/s target.
encoder = codec.H264Encoder(img.width(), img.height(), fps=FPS, bitrate=1000000, keyframe_interval=FPS)

# Microphone ring buffer. The audio driver fills it from its callback; the
# recording loop drains it. Everything is preallocated - no allocations
# happen in the audio path while recording.
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


def _audio_pending():
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
# transient that would record as a loud pop - let it pass and drop it.
time.sleep_ms(300)
_tail = _head

led = machine.LED("LED_RED")
led.on()

clock = time.clock()  # Create a clock object to track the FPS.
try:
    # audio_buffer must cover one keyframe interval of wall time (the
    # muxer flushes audio at keyframes): 128 KB is 4 s at 16 kHz.
    start = time.ticks_ms()
    with mp4.Mp4("video_audio.mp4", img.width(), img.height(), fps=FPS,
                 audio_rate=AUDIO_RATE, audio_channels=1,
                 audio_buffer=131072, encoder=encoder) as m:
        while time.ticks_diff(time.ticks_ms(), start) < RECORD_TIME * 1000:
            clock.tick()
            img = csi0.snapshot()
            ts = time.ticks_us()
            au = encoder.encode(img, timestamp_us=ts)
            m.write(au, timestamp_us=ts)

            # Drain everything the microphone captured since the last
            # frame (the muxer copies each chunk immediately).
            chunk = _audio_pending()
            while chunk is not None:
                m.write_audio(chunk)
                chunk = _audio_pending()

            print(clock.fps())

        # Fade the last captured audio out over ~30 ms so the recording
        # does not end mid-waveform with a click. The short wait ensures
        # the microphone has delivered a final chunk to fade.
        time.sleep_ms(50)
        tail = bytearray()
        chunk = _audio_pending()
        while chunk is not None:
            tail += bytes(chunk)
            chunk = _audio_pending()
        fade = min(len(tail) // 2, 480)
        base = len(tail) - fade * 2
        for i in range(fade):
            (v,) = struct.unpack_from("<h", tail, base + i * 2)
            struct.pack_into("<h", tail, base + i * 2, (v * (fade - 1 - i)) // fade)
        m.write_audio(tail)
finally:
    audio.stop_streaming()

led.off()
encoder.deinit()
print("done: video_audio.mp4")
