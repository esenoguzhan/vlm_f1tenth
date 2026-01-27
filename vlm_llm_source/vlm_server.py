#!/usr/bin/env python3
"""
VLM 驾驶策略服务器
接收小车图像，通过 LLaVA 分析，返回驾驶决策
"""

import asyncio
import json
import base64
import struct
from enum import Enum
import httpx

# Ollama API 地址
OLLAMA_URL = "http://localhost:11434/api/generate"

# VLM Prompt - Driving Strategy Analysis
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


class DrivingDecision(Enum):
    ACCELERATE = "ACCELERATE"
    DECELERATE = "DECELERATE"
    LANE_CHANGE_LEFT = "LANE_CHANGE_LEFT"
    LANE_CHANGE_RIGHT = "LANE_CHANGE_RIGHT"
    OVERTAKE = "OVERTAKE"
    KEEP_LANE = "KEEP_LANE"
    STOP = "STOP"


async def analyze_image(image_base64: str) -> dict:
    """调用 Ollama LLaVA 分析图像"""
    payload = {
        "model": "llava:7b",
        "prompt": DRIVING_PROMPT,
        "images": [image_base64],
        "stream": False,
        "options": {
            "temperature": 0.1,  # 低温度，更确定性的输出
        }
    }

    async with httpx.AsyncClient(timeout=30.0) as client:
        response = await client.post(OLLAMA_URL, json=payload)
        result = response.json()

    # 解析 VLM 输出
    try:
        response_text = result.get("response", "")
        # 尝试提取 JSON
        start = response_text.find("{")
        end = response_text.rfind("}") + 1
        if start != -1 and end > start:
            decision_data = json.loads(response_text[start:end])
            return {
                "success": True,
                "decision": decision_data.get("decision", "KEEP_LANE"),
                "confidence": decision_data.get("confidence", 0.5),
                "reason": decision_data.get("reason", ""),
            }
    except json.JSONDecodeError:
        pass

    return {
        "success": False,
        "decision": "KEEP_LANE",
        "confidence": 0.0,
        "reason": "VLM 解析失败",
    }


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    """处理来自小车的连接"""
    addr = writer.get_extra_info('peername')
    print(f"[连接] 小车已连接: {addr}")

    try:
        while True:
            # 读取消息长度 (4 bytes, big-endian)
            length_data = await reader.readexactly(4)
            msg_length = struct.unpack(">I", length_data)[0]

            # 读取图像数据
            image_data = await reader.readexactly(msg_length)
            image_base64 = base64.b64encode(image_data).decode("utf-8")

            print(f"[接收] 图像大小: {msg_length} bytes")

            # VLM 分析
            result = await analyze_image(image_base64)
            print(f"[决策] {result['decision']} (置信度: {result['confidence']:.2f}) - {result['reason']}")

            # 发送决策回小车
            response_json = json.dumps(result).encode("utf-8")
            writer.write(struct.pack(">I", len(response_json)))
            writer.write(response_json)
            await writer.drain()

    except asyncio.IncompleteReadError:
        print(f"[断开] 小车断开连接: {addr}")
    except Exception as e:
        print(f"[错误] {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def main():
    server = await asyncio.start_server(handle_client, "0.0.0.0", 9999)
    addr = server.sockets[0].getsockname()
    print(f"=" * 50)
    print(f"VLM 驾驶策略服务器已启动")
    print(f"监听地址: {addr[0]}:{addr[1]}")
    print(f"=" * 50)
    print(f"等待小车连接...")

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
