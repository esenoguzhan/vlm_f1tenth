#!/usr/bin/env python3
"""
Test script for MLX VLM driving strategy analysis
Usage: python test_mlx.py <image_path>
"""

import sys
import time
from pathlib import Path

from PIL import Image
from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

MODEL_PATH = "mlx-community/Qwen2.5-VL-3B-Instruct-8bit"

DRIVING_PROMPT = """You are an autonomous driving decision system. Analyze this road image and provide a driving strategy.

Identify key elements:
- Road conditions (clear, wet, obstacles)
- Traffic signs and signals
- Other vehicles and pedestrians
- Lane markings

Choose the most appropriate decision:
- ACCELERATE: Speed up (road ahead is clear)
- DECELERATE: Slow down (obstacle ahead or caution needed)
- LANE_CHANGE_LEFT: Change to left lane
- LANE_CHANGE_RIGHT: Change to right lane
- OVERTAKE: Overtake (slow vehicle ahead, safe to pass)
- KEEP_LANE: Maintain current lane and speed
- STOP: Stop (emergency situation)

Response format (JSON only):
{"decision": "DECISION", "confidence": 0.0-1.0, "reason": "brief reason"}"""


def test_vlm(image_path: str):
    print(f"Loading image: {image_path}")

    # Load model
    print("Loading Qwen2-VL model (first run will download ~8GB)...")
    start = time.time()
    model, processor = load(MODEL_PATH, trust_remote_code=True)
    config = load_config(MODEL_PATH, trust_remote_code=True)
    print(f"Model loaded in {time.time() - start:.1f}s")

    # Prepare image
    image = Image.open(image_path).convert("RGB")
    max_size = 512
    if max(image.size) > max_size:
        ratio = max_size / max(image.size)
        new_size = (int(image.size[0] * ratio), int(image.size[1] * ratio))
        image = image.resize(new_size, Image.LANCZOS)

    temp_path = "/tmp/vlm_test_image.jpg"
    image.save(temp_path)

    # Build prompt
    messages = [{"role": "user", "content": DRIVING_PROMPT}]
    prompt = apply_chat_template(processor, config, messages, add_generation_prompt=True)

    # Generate
    print("\nAnalyzing image...")
    start = time.time()
    output = generate(
        model,
        processor,
        prompt,
        image=temp_path,
        max_tokens=200,
        temperature=0.1,
        verbose=False
    )
    inference_time = time.time() - start

    print("\n" + "=" * 50)
    print("VLM Output:")
    print(output)
    print("=" * 50)
    print(f"\nInference time: {inference_time:.2f}s")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_mlx.py <image_path>")
        print("Example: python test_mlx.py road.jpg")
        sys.exit(1)

    test_vlm(sys.argv[1])
