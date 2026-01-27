#!/usr/bin/env python3
"""
MLX VLM Driving Strategy Server with Preview
Uses Phi-3.5-vision to analyze images and return driving decisions
Optimized for Apple Silicon, faster inference

This version:
- Crops to keep only the bottom 2/3 of the image
- Shows a live preview window of received images
"""

import asyncio
import json
import struct
import time
from io import BytesIO

import cv2
import numpy as np
from PIL import Image
from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config


# Model configuration
MODEL_PATH = "mlx-community/Qwen2.5-VL-7B-Instruct-4bit"

# RC car indoor racing track prompt
DRIVING_PROMPT = "Is there a black RC car on the ground? Is it on your RIGHT or LEFT or CENTRE?"


class MLXVLMServer:
    def __init__(self):
        print("Loading Qwen2.5-VL-3B model...")
        start = time.time()

        # Load model and processor
        self.model, self.processor = load(MODEL_PATH, trust_remote_code=True, use_fast=False)
        self.config = load_config(MODEL_PATH, trust_remote_code=True)

        print(f"Model loaded in {time.time() - start:.1f}s")
        print(f"Model: {MODEL_PATH}")

        # Create preview window
        cv2.namedWindow("Video Preview", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video Preview", 640, 360)

    def analyze_image(self, image_bytes: bytes) -> dict:
        """Analyze image and return driving decision"""
        start = time.time()

        try:
            # Convert bytes to PIL Image
            image = Image.open(BytesIO(image_bytes)).convert("RGB")

            # Crop to keep only the bottom 2/3
            width, height = image.size
            image = image.crop((0, height // 3, width, height))

            # Show preview (convert PIL to OpenCV format)
            preview_img = np.array(image)
            preview_img = cv2.cvtColor(preview_img, cv2.COLOR_RGB2BGR)
            cv2.imshow("Video Preview", preview_img)
            cv2.waitKey(1)  # Required to update the window

            # Resize image to speed up processing
            max_size = 512
            if max(image.size) > max_size:
                ratio = max_size / max(image.size)
                new_size = (int(image.size[0] * ratio), int(image.size[1] * ratio))
                image = image.resize(new_size, Image.LANCZOS)

            # Save temporary image
            temp_path = "/tmp/vlm_temp_image.jpg"
            image.save(temp_path)

            # Build messages with image reference
            messages = [
                {
                    "role": "user",
                    "content": [
                        {"type": "image", "image": temp_path},
                        {"type": "text", "text": DRIVING_PROMPT}
                    ]
                }
            ]

            # Apply chat template
            prompt = apply_chat_template(
                self.processor,
                self.config,
                messages,
                add_generation_prompt=True,
                num_images=1
            )

            # Generate response
            output = generate(
                self.model,
                self.processor,
                prompt,
                image=temp_path,
                max_tokens=20,
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
                "decision": "centre",
                "confidence": 0.0,
                "reason": f"Analysis failed: {str(e)}",
                "inference_time": 0
            }

    def _parse_response(self, response: str, inference_time: float) -> dict:
        """Parse VLM output - only extract left or right"""
        response_lower = response.lower()

        # Extract left or right from response
        if "left" in response_lower:
            position = "left"
        elif "right" in response_lower:
            position = "right"
        elif "center" in response_lower:
            position = "center"
        else:
            position = "stay"

        return {
            "success": position != "unknown",
            "decision": position,
            "inference_time": inference_time
        }

    def cleanup(self):
        """Clean up OpenCV windows"""
        cv2.destroyAllWindows()


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
            print(f"[Decision] {result['decision']}")

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
    print("MLX VLM Server with Preview (Bottom 2/3 Crop)")
    print(f"Listening on: {addr[0]}:{addr[1]}")
    print("=" * 50)
    print("Waiting for vehicle connection...")
    print("Press 'q' in the preview window to quit")

    try:
        async with tcp_server:
            await tcp_server.serve_forever()
    finally:
        server.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
