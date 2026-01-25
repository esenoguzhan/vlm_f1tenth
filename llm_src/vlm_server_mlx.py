#!/usr/bin/env python3
"""
MLX VLM Driving Strategy Server
Uses Phi-3.5-vision to analyze images and return driving decisions
Optimized for Apple Silicon, faster inference
"""

import asyncio
import json
import struct
import time
from io import BytesIO

from PIL import Image
from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

# Model configuration
MODEL_PATH = "mlx-community/Qwen2.5-VL-3B-Instruct-8bit"

# RC car indoor racing track prompt
DRIVING_PROMPT = """Indoor robot car track with white barriers on sides.

Look for:
1. Small black robot car (toy car with wheels) ahead on the gray floor
2. Person making STOP gesture: person's open hand/palm clearly facing the camera, fingers spread

IMPORTANT: Only set hand_gesture to "stop" if you clearly see a person's open palm facing you. Otherwise set "none".

Answer:
1. Robot car ahead? Where? (left / right / center / none)
2. Clear stop hand gesture visible? (stop / none) - must be open palm facing camera
3. Path direction? (straight / left / right)

Decision rules:
- STOP: Clear stop hand gesture (person's open palm facing camera)
- GO_LEFT: Robot car on RIGHT side
- GO_RIGHT: Robot car on LEFT side
- KEEP_CENTER: Robot car in CENTER
- ACCELERATE: No car, no stop gesture, straight path
- DECELERATE: Curve ahead

JSON:
{"car_position": "none/left/right/center", "hand_gesture": "none/stop", "path": "straight/left/right", "decision": "ACTION"}"""


class MLXVLMServer:
    def __init__(self):
        print("Loading Qwen2.5-VL-3B model...")
        start = time.time()

        # Load model and processor
        self.model, self.processor = load(MODEL_PATH, trust_remote_code=True, use_fast=False)
        self.config = load_config(MODEL_PATH, trust_remote_code=True)

        print(f"Model loaded in {time.time() - start:.1f}s")
        print(f"Model: {MODEL_PATH}")

    def analyze_image(self, image_bytes: bytes) -> dict:
        """Analyze image and return driving decision"""
        start = time.time()

        try:
            # Convert bytes to PIL Image
            image = Image.open(BytesIO(image_bytes)).convert("RGB")

            # Resize image to speed up processing
            max_size = 512
            if max(image.size) > max_size:
                ratio = max_size / max(image.size)
                new_size = (int(image.size[0] * ratio), int(image.size[1] * ratio))
                image = image.resize(new_size, Image.LANCZOS)

            # Save temporary image
            temp_path = "/tmp/vlm_temp_image.jpg"
            image.save(temp_path)

            # Build messages
            messages = [{"role": "user", "content": DRIVING_PROMPT}]

            # Apply chat template
            prompt = apply_chat_template(
                self.processor,
                self.config,
                messages,
                add_generation_prompt=True
            )

            # Generate response
            output = generate(
                self.model,
                self.processor,
                prompt,
                image=temp_path,
                max_tokens=200,
                temperature=0.1,
                verbose=False
            )

            inference_time = time.time() - start
            print(f"[Inference] {inference_time:.2f}s")

            # Parse output (handle GenerationResult object)
            response_text = output.text if hasattr(output, 'text') else str(output)
            print(f"[Raw Output] {response_text[:200]}")  # Debug: show raw output
            return self._parse_response(response_text, inference_time)

        except Exception as e:
            print(f"[Error] Analysis failed: {e}")
            return {
                "success": False,
                "decision": "KEEP_LANE",
                "confidence": 0.0,
                "reason": f"Analysis failed: {str(e)}",
                "inference_time": 0
            }

    def _parse_response(self, response: str, inference_time: float) -> dict:
        """Parse VLM output"""
        try:
            # Try to extract JSON
            start = response.find("{")
            end = response.rfind("}") + 1
            if start != -1 and end > start:
                decision_data = json.loads(response[start:end])
                return {
                    "success": True,
                    "decision": decision_data.get("decision", "ACCELERATE"),
                    "confidence": 1.0,
                    "car_position": decision_data.get("car_position", "none"),
                    "hand_gesture": decision_data.get("hand_gesture", "none"),
                    "path": decision_data.get("path", "straight"),
                    "reason": f"car:{decision_data.get('car_position','none')} gesture:{decision_data.get('hand_gesture','none')}",
                    "inference_time": inference_time
                }
        except json.JSONDecodeError:
            pass

        # JSON parsing failed, try to extract decision from text
        decisions = ["STOP", "GO_LEFT", "GO_RIGHT", "KEEP_CENTER",
                     "ACCELERATE", "DECELERATE"]
        for decision in decisions:
            if decision in response.upper():
                return {
                    "success": True,
                    "decision": decision,
                    "confidence": 0.5,
                    "reason": response[:100],
                    "inference_time": inference_time
                }

        return {
            "success": False,
            "decision": "KEEP_LANE",
            "confidence": 0.0,
            "reason": "Failed to parse response",
            "inference_time": inference_time
        }


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, server: MLXVLMServer):
    """Handle vehicle connection"""
    addr = writer.get_extra_info('peername')
    print(f"[Connected] Vehicle connected: {addr}")

    try:
        while True:
            # Read message length (4 bytes, big-endian)
            length_data = await reader.readexactly(4)
            msg_length = struct.unpack(">I", length_data)[0]

            # Read image data
            image_data = await reader.readexactly(msg_length)
            print(f"[Received] Image size: {msg_length} bytes")

            # VLM analysis
            result = server.analyze_image(image_data)
            print(f"[Decision] {result['decision']} | car: {result.get('car_position', 'N/A')} | gesture: {result.get('hand_gesture', 'N/A')} | path: {result.get('path', 'N/A')}")

            # Send decision back to vehicle
            response_json = json.dumps(result).encode("utf-8")
            writer.write(struct.pack(">I", len(response_json)))
            writer.write(response_json)
            await writer.drain()

    except asyncio.IncompleteReadError:
        print(f"[Disconnected] Vehicle disconnected: {addr}")
    except Exception as e:
        print(f"[Error] {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def main():
    # Initialize VLM server
    server = MLXVLMServer()

    # Start TCP server
    tcp_server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, server),
        "0.0.0.0",
        9999
    )

    addr = tcp_server.sockets[0].getsockname()
    print("=" * 50)
    print("MLX VLM Driving Strategy Server")
    print(f"Listening on: {addr[0]}:{addr[1]}")
    print("=" * 50)
    print("Waiting for vehicle connection...")

    async with tcp_server:
        await tcp_server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
