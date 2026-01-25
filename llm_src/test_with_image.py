#!/usr/bin/env python3
"""
测试脚本：用本地图片测试 VLM 驾驶策略分析
使用方法: python test_with_image.py <图片路径>
"""

import sys
import base64
import httpx
import json

OLLAMA_URL = "http://localhost:11434/api/generate"

DRIVING_PROMPT = """You are an autonomous driving decision system. Analyze this road image and provide a driving strategy.

Choose the most appropriate decision from the following options:
- ACCELERATE: Speed up (road ahead is clear)
- DECELERATE: Slow down (obstacle ahead or caution needed)
- LANE_CHANGE_LEFT: Change to left lane
- LANE_CHANGE_RIGHT: Change to right lane
- OVERTAKE: Overtake (slow vehicle ahead, safe to pass)
- KEEP_LANE: Maintain current lane and speed
- STOP: Stop (emergency situation)

Response format (JSON):
{"decision": "DECISION", "confidence": 0.0-1.0, "reason": "brief reason"}

Return only JSON, no other content."""


def test_vlm(image_path: str):
    print(f"读取图片: {image_path}")

    with open(image_path, "rb") as f:
        image_base64 = base64.b64encode(f.read()).decode("utf-8")

    print("正在调用 VLM 分析...")

    payload = {
        "model": "llava:7b",
        "prompt": DRIVING_PROMPT,
        "images": [image_base64],
        "stream": False,
        "options": {"temperature": 0.1},
    }

    response = httpx.post(OLLAMA_URL, json=payload, timeout=60.0)
    result = response.json()

    print("\n" + "=" * 50)
    print("VLM 原始输出:")
    print(result.get("response", "无输出"))
    print("=" * 50)

    # 尝试解析 JSON
    response_text = result.get("response", "")
    start = response_text.find("{")
    end = response_text.rfind("}") + 1
    if start != -1 and end > start:
        try:
            decision = json.loads(response_text[start:end])
            print("\n解析结果:")
            print(f"  决策: {decision.get('decision')}")
            print(f"  置信度: {decision.get('confidence')}")
            print(f"  原因: {decision.get('reason')}")
        except json.JSONDecodeError:
            print("JSON 解析失败")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("使用方法: python test_with_image.py <图片路径>")
        print("示例: python test_with_image.py road.jpg")
        sys.exit(1)

    test_vlm(sys.argv[1])
