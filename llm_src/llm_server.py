#!/usr/bin/env python3
"""Voice Control for RC Car - MLX Qwen2 + Whisper"""

import asyncio, json, struct, tempfile, os
import numpy as np
import sounddevice as sd
import whisper
from mlx_lm import load, generate

QWEN2_MODEL = "mlx-community/Qwen2.5-7B-Instruct-4bit"
SAMPLE_RATE = 16000
RECORD_SECONDS = 3

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


class LLMServer:
    def __init__(self):
        print("Loading Whisper...")
        self.whisper = whisper.load_model("base")
        print("Loading Qwen2...")
        self.llm, self.tokenizer = load(QWEN2_MODEL)
        self.car_writer = None
        print("\nReady! [Enter] to speak, 'q' to quit")

    def record(self):
        print(f"Recording {RECORD_SECONDS}s...")
        audio = sd.rec(int(RECORD_SECONDS * SAMPLE_RATE),
                       samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()
        return audio.flatten()

    def transcribe(self, audio):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            path = f.name
            import scipy.io.wavfile as wav
            wav.write(path, SAMPLE_RATE, (audio * 32767).astype(np.int16))
        try:
            return self.whisper.transcribe(path, language="en", fp16=False)["text"].strip()
        finally:
            os.unlink(path)

    def parse_command(self, text):
        if not text:
            return {"action": "STOP", "reason": "no input"}

        prompt = PROMPT.format(text=text)
        formatted = self.tokenizer.apply_chat_template(
            [{"role": "user", "content": prompt}], tokenize=False, add_generation_prompt=True)
        response = generate(self.llm, self.tokenizer, prompt=formatted, max_tokens=50, verbose=False)

        try:
            start, end = response.find("{"), response.rfind("}") + 1
            if start != -1 and end > start:
                return json.loads(response[start:end])
        except:
            pass

        for action in ["GO_LEFT", "GO_RIGHT", "ACCELERATE", "DECELERATE", "STOP"]:
            if action in response.upper():
                return {"action": action, "reason": "keyword"}
        return {"action": "STOP", "reason": "unknown"}

    async def send_to_car(self, cmd):
        if not self.car_writer:
            print("Car not connected")
            return
        try:
            data = json.dumps(cmd).encode()
            self.car_writer.write(struct.pack(">I", len(data)) + data)
            await self.car_writer.drain()
            print(f"Sent: {cmd['action']}")
        except:
            self.car_writer = None

    async def handle_car(self, reader, writer):
        print("Car connected!")
        self.car_writer = writer
        try:
            while self.car_writer:
                await asyncio.sleep(1)
        finally:
            self.car_writer = None
            writer.close()

    async def voice_loop(self):
        loop = asyncio.get_event_loop()
        while True:
            cmd = await loop.run_in_executor(None, lambda: input("\n[Enter] record, 'q' quit: "))
            if cmd.lower() == 'q':
                break
            audio = await loop.run_in_executor(None, self.record)
            text = await loop.run_in_executor(None, self.transcribe, audio)
            print(f"Heard: {text}")
            if text:
                result = await loop.run_in_executor(None, self.parse_command, text)
                print(f"Action: {result['action']} ({result.get('reason', '')})")
                await self.send_to_car(result)

    async def run(self):
        server = await asyncio.start_server(self.handle_car, "0.0.0.0", 9999)
        print("Listening on port 9999")
        async with server:
            await asyncio.gather(server.serve_forever(), self.voice_loop())


if __name__ == "__main__":
    asyncio.run(LLMServer().run())
