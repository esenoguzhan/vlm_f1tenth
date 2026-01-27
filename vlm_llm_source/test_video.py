#!/usr/bin/env python3
"""
Video analysis script for racing track
Extracts frames from video and analyzes with VLM
"""

import sys
import time
import json
import cv2
from pathlib import Path
from PIL import Image
from io import BytesIO

from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

MODEL_PATH = "mlx-community/Qwen2.5-VL-3B-Instruct-8bit"

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


def analyze_video(video_path: str, frame_interval: int = 30):
    """Analyze video frames with VLM"""

    print(f"Loading model: {MODEL_PATH}")
    model, processor = load(MODEL_PATH, trust_remote_code=True)
    config = load_config(MODEL_PATH, trust_remote_code=True)
    print("Model loaded.\n")

    # Open video
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"Video: {video_path}")
    print(f"FPS: {fps:.1f}, Total frames: {total_frames}")
    print(f"Analyzing every {frame_interval} frames (~{frame_interval/fps:.1f}s interval)")
    print("=" * 60)

    frame_num = 0
    results = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_num % frame_interval == 0:
            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame_rgb)

            # Resize for faster processing
            max_size = 512
            if max(image.size) > max_size:
                ratio = max_size / max(image.size)
                new_size = (int(image.size[0] * ratio), int(image.size[1] * ratio))
                image = image.resize(new_size, Image.LANCZOS)

            # Save temp image
            temp_path = "/tmp/vlm_video_frame.jpg"
            image.save(temp_path)

            # Build prompt
            messages = [{"role": "user", "content": DRIVING_PROMPT}]
            prompt = apply_chat_template(processor, config, messages, add_generation_prompt=True)

            # Analyze
            start = time.time()
            output = generate(
                model, processor, prompt,
                image=temp_path,
                max_tokens=150,
                temperature=0.1,
                verbose=False
            )
            inference_time = time.time() - start

            # Parse result
            response_text = output.text if hasattr(output, 'text') else str(output)

            # Extract JSON
            try:
                start_idx = response_text.find("{")
                end_idx = response_text.rfind("}") + 1
                if start_idx != -1 and end_idx > start_idx:
                    result = json.loads(response_text[start_idx:end_idx])
                else:
                    result = {"decision": "UNKNOWN", "scene": response_text[:100]}
            except:
                result = {"decision": "UNKNOWN", "scene": response_text[:100]}

            timestamp = frame_num / fps
            result["frame"] = frame_num
            result["timestamp"] = f"{timestamp:.1f}s"
            result["inference_time"] = f"{inference_time:.2f}s"
            results.append(result)

            # Print result
            scene_text = str(result.get('scene', ''))[:40]
            decision = str(result.get('decision', 'UNKNOWN'))
            print(f"[{timestamp:5.1f}s] Frame {frame_num:4d} | {decision:15s} | {scene_text}...")

        frame_num += 1

    cap.release()

    print("=" * 60)
    print(f"Analyzed {len(results)} frames")

    # Summary
    decisions = {}
    for r in results:
        d = r.get("decision", "UNKNOWN")
        decisions[d] = decisions.get(d, 0) + 1

    print("\nDecision summary:")
    for d, count in sorted(decisions.items(), key=lambda x: -x[1]):
        print(f"  {d}: {count} times ({100*count/len(results):.0f}%)")

    return results


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3.11 test_video.py <video_path> [frame_interval]")
        print("Example: python3.11 test_video.py video.mp4 30")
        sys.exit(1)

    video_path = sys.argv[1]
    frame_interval = int(sys.argv[2]) if len(sys.argv) > 2 else 30

    analyze_video(video_path, frame_interval)
