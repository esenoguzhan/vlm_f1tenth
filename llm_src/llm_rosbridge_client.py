#!/usr/bin/env python3
"""
Standalone PC Control Script (No ROS Installation Required)
-----------------------------------------------------------
This script runs entirely on your computer. It listens to your microphone, 
processes speech using Google STT and a local LLM, and sends commands 
to the robot over Wi-Fi using WebSockets.

IT IS NOT A ROS NODE. It behaves like a remote control.

Usage:
1. Start rosbridge on robot: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
2. Run this script on PC: python llm_rosbridge_client.py

Dependencies:
  pip install sounddevice numpy SpeechRecognition roslibpy torch transformers accelerate
  
  (If SpeechRecognition fails, try: pip install --upgrade setuptools wheel)
"""

import json
import time
import threading
import numpy as np
import sounddevice as sd
import speech_recognition as sr
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
import roslibpy

# ==================== Configuration ====================
ROBOT_IP = "10.183.170.108"  # Ubuntu robot car IP
ROSBRIDGE_PORT = 9090        # rosbridge default port

# Use smaller 1.5B model for fast local inference on PC
QWEN2_MODEL = "Qwen/Qwen2.5-1.5B-Instruct"  
LLM_TOPIC = "/llm_vel"       # Topic to publish decisions
MSG_TYPE = "std_msgs/String"
WAKE_WORD = "luna"

# Audio settings
SAMPLE_RATE = 16000
CHUNK_MS = 200     # Smaller chunk for smoother VAD
SILENCE_THRESHOLD = 0.003
SILENCE_CHUNKS = 5  # ~1 second of silence (5 * 200ms)
MAX_RECORD_SECONDS = 10  # Max recording length

# LLM Prompt
PROMPT = """You control a car. Parse voice command into action.

Actions: GO_LEFT, GO_RIGHT, ACCELERATE, DECELERATE, STOP

User said: "{text}"

Return JSON only: {{"action": "ACTION", "reason": "brief"}}"""


def record_until_silence():
    """Record audio using a persistent input stream to avoid gaps/crashes"""
    chunk_size = int(SAMPLE_RATE * CHUNK_MS / 1000)
    audio_chunks = []
    silence_count = 0
    speech_started = False
    
    print("\n[Listening] ...", end="", flush=True)

    # Use a persistent stream to prevent driver crashes and audio gaps
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='float32', blocksize=chunk_size) as stream:
        while True:
            # Read chunk
            chunk, overflowed = stream.read(chunk_size)
            if overflowed:
                print("!", end="", flush=True)
            
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
                # Keep silence at the end for natural cutoff
                audio_chunks.append(chunk)  
                silence_count += 1
                if silence_count >= SILENCE_CHUNKS:
                    print(" [Done]")
                    break
            else:
                # Not started yet, check max duration timeout or just wait
                if len(audio_chunks) * (CHUNK_MS/1000) > MAX_RECORD_SECONDS:
                     break
                print(".", end="", flush=True)
                
            # Limit max recording length
            if len(audio_chunks) * (CHUNK_MS/1000) > MAX_RECORD_SECONDS:
                print(" [Timeout]")
                break

    if not speech_started:
        print()
        return None

    return np.concatenate(audio_chunks)


class MPCControlScript:
    def __init__(self):
        print("=" * 50)
        print("PC Control Script - Voice -> LLM -> Robot")
        print("=" * 50)

        # Load Hugging Face LLM
        print(f"\n[Loading] {QWEN2_MODEL}...")
        try:
            # Load tokenizer
            self.tokenizer = AutoTokenizer.from_pretrained(QWEN2_MODEL, trust_remote_code=True)
            
            # Load model (automatically uses GPU if available via device_map="auto")
            self.model = AutoModelForCausalLM.from_pretrained(
                QWEN2_MODEL, 
                device_map="auto", 
                dtype="auto", 
                trust_remote_code=True
            )
            print(f"[Done] LLM loaded on {self.model.device}")
        except Exception as e:
            print(f"[Error] Failed to load model: {e}")
            print("Make sure you have installed: pip install torch transformers accelerate")
            return

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
        
        # Initialize Speech Recognizer
        self.recognizer = sr.Recognizer()

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

    def transcribe_audio(self, audio_data_np):
        """Convert numpy audio to SpeechRecognition AudioData and transcribe"""
        try:
            # Check amplitude
            max_val = np.max(np.abs(audio_data_np))
            print(f" [Max Amp: {max_val:.4f}]", end="", flush=True)
            
            if max_val == 0:
                print(" [Silent]")
                return None

            # Normalize audio (Boost volume to max)
            # This helps significantly if the mic input is quiet
            audio_norm = audio_data_np / max_val
            
            # Convert float32 [-1, 1] to int16 [-32768, 32767]
            audio_int16 = (audio_norm * 32767).astype(np.int16)
            audio_bytes = audio_int16.tobytes()
            
            # Create AudioData instance
            audio_source = sr.AudioData(audio_bytes, SAMPLE_RATE, 2)
            
            print(" [Transcribing] ...", end="", flush=True)
            text = self.recognizer.recognize_google(audio_source).lower()
            return text
            
        except sr.UnknownValueError:
            print("\n[?] Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"\n[Error] Speech API error: {e}")
            return None
        except Exception as e:
            print(f"\n[Error] Transcription error: {e}")
            return None

    def generate_decision(self, command: str) -> dict:
        """Generate driving decision using local LLM (Transformers)"""
        prompt = PROMPT.format(text=command)

        # Apply chat template
        messages = [{"role": "user", "content": prompt}]
        text_input = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        
        # Tokenize and generate
        model_inputs = self.tokenizer([text_input], return_tensors="pt").to(self.model.device)
        generated_ids = self.model.generate(
            **model_inputs,
            max_new_tokens=50,
            do_sample=False  # Deterministic output
        )
        
        # Decode response (remove the prompt part)
        generated_ids = [
            output_ids[len(input_ids):] for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
        ]
        response = self.tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]

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
        """Main loop: Listen (sounddevice) -> Transcribe (Google) -> Filter (Luna) -> LLM -> Publish"""
        print("\n" + "=" * 50)
        print("Voice Control - Local PC")
        print(f"Wake Word: '{WAKE_WORD.upper()}' (e.g., 'Luna, go left')")
        print("Speak until you are done (auto-silence detection).")
        print("=" * 50 + "\n")

        while True:
            try:
                # 1. Listen
                audio_np = record_until_silence()
                
                if audio_np is None:
                    continue

                # 2. Transcribe
                text = self.transcribe_audio(audio_np)
                
                if not text:
                    continue
                
                print(f"\n[Heard] \"{text}\"")

                # 3. Check for Wake Word
                if WAKE_WORD not in text:
                    print(f"[Ignored] Wake word '{WAKE_WORD}' not found.")
                    continue
                
                # Split and take command after wake word
                parts = text.split(WAKE_WORD, 1)
                command = parts[1].strip()
                
                if not command:
                    print("[Ignored] No command after wake word.")
                    continue
                
                print(f"[Command] \"{command}\"")

                # 4. LLM Decision
                result = self.generate_decision(command)
                print(f">>> Action: {result['action']} | Reason: {result.get('reason', '')}")

                # 5. Publish
                self.publish_decision(result)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break

    def close(self):
        """Close connection"""
        self.publisher.unadvertise()
        self.ros_client.terminate()
        print("[Closed] Connection terminated")


def main():
    script = MPCControlScript()

    # Connect to ROS in background
    connect_thread = threading.Thread(target=script.connect)
    connect_thread.daemon = True
    connect_thread.start()

    # Wait for connection
    print("[Waiting] Connecting to robot...")
    timeout = 10
    start_time = time.time()
    while not script.connected and (time.time() - start_time) < timeout:
        time.sleep(0.5)

    if not script.connected:
        print("[Error] Connection timeout. Check Robot IP and rosbridge.")
        print(f"Robot IP: {ROBOT_IP}")
        return

    try:
        # Run voice loop in main thread
        script.voice_loop()
    finally:
        script.close()


if __name__ == "__main__":
    main()
