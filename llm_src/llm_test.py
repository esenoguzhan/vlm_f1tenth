#!/usr/bin/env python3
"""Voice Control Test - Wait for speech to finish before processing"""

import tempfile, os, json
import numpy as np
import sounddevice as sd
import whisper
from mlx_lm import load, generate

QWEN2_MODEL = "mlx-community/Qwen2.5-7B-Instruct-4bit"
SAMPLE_RATE = 16000
CHUNK_MS = 500  # Check every 500ms
SILENCE_THRESHOLD = 0.003
SILENCE_CHUNKS = 2  # Wait for 1s silence to consider speech ended
MAX_RECORD_SECONDS = 10  # Max recording length

PROMPT = """You control a car. Parse voice command into action.

Actions: GO_LEFT, GO_RIGHT, ACCELERATE, DECELERATE, STOP

Examples:
- "go left" / "move left" / "turn left" -> GO_LEFT
- "go right" / "move right" -> GO_RIGHT
- "speed up" / "faster" / "go" -> ACCELERATE
- "slow down" / "careful" -> DECELERATE
- "stop" / "halt" / "brake" -> STOP

User said: "{text}"

Return JSON only: {{"action": "ACTION", "reason": "brief"}}"""


def record_until_silence():
    """Record audio until user stops speaking"""
    chunk_size = int(SAMPLE_RATE * CHUNK_MS / 1000)
    max_chunks = int(MAX_RECORD_SECONDS * 1000 / CHUNK_MS)

    audio_chunks = []
    silence_count = 0
    speech_started = False

    print("Listening...", end="", flush=True)

    for _ in range(max_chunks):
        chunk = sd.rec(chunk_size, samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()
        chunk = chunk.flatten()

        level = np.abs(chunk).mean()
        has_speech = level > SILENCE_THRESHOLD

        if has_speech:
            if not speech_started:
                speech_started = True
                print(" [Speaking]", end="", flush=True)
            silence_count = 0
            audio_chunks.append(chunk)
        elif speech_started:
            audio_chunks.append(chunk)  # Keep silence in recording
            silence_count += 1
            if silence_count >= SILENCE_CHUNKS:
                print(" [Done]")
                break
        else:
            print(".", end="", flush=True)

    if not speech_started:
        print()
        return None

    return np.concatenate(audio_chunks)


def main():
    print("Loading Whisper...")
    whisper_model = whisper.load_model("medium")

    print("Loading Qwen2...")
    llm, tokenizer = load(QWEN2_MODEL)

    print("\n" + "="*50)
    print("Ready! Speak anytime. (Ctrl+C to quit)")
    print("="*50 + "\n")

    while True:
        try:
            audio = record_until_silence()

            if audio is None:
                continue

            # Transcribe
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                path = f.name
                import scipy.io.wavfile as wav
                wav.write(path, SAMPLE_RATE, (audio * 32767).astype(np.int16))

            text = whisper_model.transcribe(path, language="en", fp16=False)["text"].strip()
            os.unlink(path)

            if not text:
                print("[No speech recognized]\n")
                continue

            print(f"You said: \"{text}\"")

            # LLM parse
            prompt = PROMPT.format(text=text)
            formatted = tokenizer.apply_chat_template(
                [{"role": "user", "content": prompt}], tokenize=False, add_generation_prompt=True)
            response = generate(llm, tokenizer, prompt=formatted, max_tokens=50, verbose=False)

            # Parse result
            try:
                start, end = response.find("{"), response.rfind("}") + 1
                if start != -1 and end > start:
                    result = json.loads(response[start:end])
                    print(f">>> Action: {result['action']} | Reason: {result.get('reason', '')}\n")
                    continue
            except:
                pass

            for action in ["GO_LEFT", "GO_RIGHT", "ACCELERATE", "DECELERATE", "STOP"]:
                if action in response.upper():
                    print(f">>> Action: {action}\n")
                    break
            else:
                print(f">>> Action: UNKNOWN (raw: {response})\n")

        except KeyboardInterrupt:
            print("\nStopped.")
            break


if __name__ == "__main__":
    main()
