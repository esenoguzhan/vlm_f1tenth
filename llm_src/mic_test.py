#!/usr/bin/env python3
"""Test microphone input levels"""
import numpy as np
import sounddevice as sd

SAMPLE_RATE = 16000

print("Testing microphone... Speak now!")
print("(Ctrl+C to stop)\n")

while True:
    try:
        audio = sd.rec(int(1 * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()
        level = np.abs(audio).mean()
        bar = "=" * int(level * 500)
        print(f"Level: {level:.4f} |{bar}")
    except KeyboardInterrupt:
        break
