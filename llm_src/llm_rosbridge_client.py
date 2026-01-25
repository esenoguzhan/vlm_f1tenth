#!/usr/bin/env python3
"""
LLM ROS2 Control Client - Mac Side (All-in-one)
Voice control: Whisper + Qwen2.5 + rosbridge -> /llm_vel topic

Usage:
1. Start rosbridge on robot: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
2. Run this script: python llm_rosbridge_client.py
"""

import json
import time
import threading
import tempfile
import os
import numpy as np
import sounddevice as sd
import whisper
from mlx_lm import load, generate
import roslibpy

# ==================== Configuration ====================
ROBOT_IP = "10.183.170.108"  # Ubuntu robot car IP
ROSBRIDGE_PORT = 9090        # rosbridge default port

QWEN2_MODEL = "mlx-community/Qwen2.5-7B-Instruct-4bit"
LLM_TOPIC = "/llm_vel"       # Topic to publish decisions
MSG_TYPE = "std_msgs/String"

# Audio settings (same as llm_test.py)
SAMPLE_RATE = 16000
CHUNK_MS = 500  # Check every 500ms
SILENCE_THRESHOLD = 0.003
SILENCE_CHUNKS = 2  # Wait for 1s silence to consider speech ended
MAX_RECORD_SECONDS = 10  # Max recording length

# LLM Prompt (same as llm_test.py)
PROMPT = """You control a car. Parse voice command into action.

Actions: GO_LEFT, GO_RIGHT, ACCELERATE, DECELERATE, STOP

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


class LLMRosbridgeClient:
    def __init__(self):
        print("=" * 50)
        print("LLM Rosbridge Client (Voice Control)")
        print("=" * 50)

        # Load Whisper
        print("\n[Loading] Whisper model...")
        self.whisper_model = whisper.load_model("medium")
        print("[Done] Whisper loaded")

        # Load MLX LLM
        print(f"\n[Loading] {QWEN2_MODEL}...")
        self.llm, self.tokenizer = load(QWEN2_MODEL)
        print("[Done] LLM loaded")

        # Connect to rosbridge
        print(f"\n[Connecting] rosbridge ws://{ROBOT_IP}:{ROSBRIDGE_PORT}")
        self.ros_client = roslibpy.Ros(host=ROBOT_IP, port=ROSBRIDGE_PORT)
        self.ros_client.on_ready(self.on_connected)
        self.connected = False

        # Create publisher
        self.publisher = roslibpy.Topic(
            self.ros_client,
            LLM_TOPIC,
            MSG_TYPE
        )

    def on_connected(self):
        self.connected = True
        print(f"[Done] Connected to robot rosbridge")
        print(f"[Topic] Publishing to: {LLM_TOPIC}")

    def connect(self):
        """Connect to rosbridge"""
        try:
            self.ros_client.run()
        except Exception as e:
            print(f"[Error] Connection failed: {e}")
            return False
        return True

    def transcribe(self, audio):
        """Transcribe audio using Whisper"""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            path = f.name
            import scipy.io.wavfile as wav
            wav.write(path, SAMPLE_RATE, (audio * 32767).astype(np.int16))
        try:
            return self.whisper_model.transcribe(path, language="en", fp16=False)["text"].strip()
        finally:
            os.unlink(path)

    def generate_decision(self, command: str) -> dict:
        """Generate driving decision using local LLM"""
        prompt = PROMPT.format(text=command)

        # Apply chat template
        formatted = self.tokenizer.apply_chat_template(
            [{"role": "user", "content": prompt}],
            tokenize=False,
            add_generation_prompt=True
        )

        # Generate response
        response = generate(
            self.llm,
            self.tokenizer,
            prompt=formatted,
            max_tokens=50,
            verbose=False
        )

        # Parse JSON
        try:
            start = response.find("{")
            end = response.rfind("}") + 1
            if start != -1 and end > start:
                return json.loads(response[start:end])
        except json.JSONDecodeError:
            pass

        # Fallback: keyword matching
        response_upper = response.upper()
        for action in ["GO_LEFT", "GO_RIGHT", "ACCELERATE", "DECELERATE", "STOP"]:
            if action in response_upper:
                return {"action": action, "reason": "keyword match"}

        return {"action": "STOP", "reason": "parse failed"}

    def publish_decision(self, result: dict):
        """Publish decision to ROS2 topic"""
        if not self.connected:
            print("[Warning] Not connected to rosbridge")
            return False

        msg = roslibpy.Message({'data': json.dumps(result)})
        self.publisher.publish(msg)
        print(f"[Published] {result['action']} -> {LLM_TOPIC}")
        return True

    def voice_loop(self):
        """Voice control loop: listen -> transcribe -> LLM -> publish"""
        print("\n" + "=" * 50)
        print("Voice Control Mode - Speak to control the car")
        print("Press Ctrl+C to quit")
        print("=" * 50 + "\n")

        while True:
            try:
                # Record audio
                audio = record_until_silence()

                if audio is None:
                    continue

                # Transcribe
                text = self.transcribe(audio)

                if not text:
                    print("[No speech recognized]\n")
                    continue

                print(f"You said: \"{text}\"")

                # LLM parse
                result = self.generate_decision(text)
                print(f">>> Action: {result['action']} | Reason: {result.get('reason', '')}")

                # Publish to ROS2
                self.publish_decision(result)
                print()

            except KeyboardInterrupt:
                print("\nExiting...")
                break

    def close(self):
        """Close connection"""
        self.publisher.unadvertise()
        self.ros_client.terminate()
        print("[Closed] Connection terminated")


def main():
    client = LLMRosbridgeClient()

    # Connect in background thread
    connect_thread = threading.Thread(target=client.connect)
    connect_thread.daemon = True
    connect_thread.start()

    # Wait for connection
    print("[Waiting] Connecting...")
    timeout = 10
    start_time = time.time()
    while not client.connected and (time.time() - start_time) < timeout:
        time.sleep(0.5)

    if not client.connected:
        print("[Error] Connection timeout. Please check:")
        print(f"  1. Robot IP is correct: {ROBOT_IP}")
        print(f"  2. rosbridge is running: ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
        return

    try:
        client.voice_loop()
    finally:
        client.close()


if __name__ == "__main__":
    main()
