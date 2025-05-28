# socket_server.py
import json
import asyncio
import websockets
import base64
import numpy as np
import cv2
from abnormal_model.abnormal_predict import predict_from_bytes as predict_abnormal
from type_model.type_predict import predict_from_bytes as predict_type

async def handle_client(websocket, path):
    async for message in websocket:
        try:
            data = json.loads(message)
            image_b64 = data.get("image")
            if not image_b64:
                await websocket.send(json.dumps({"error": "Image data not provided"}))
                continue

            # 이미지 디코딩
            image_bytes = base64.b64decode(image_b64)

            # 1️⃣ 이상 탐지 예측
            abnormal_result = predict_abnormal(image_bytes)
            if abnormal_result["abnormal"] == True:
                await websocket.send(json.dumps(abnormal_result))
                continue

            # 2️⃣ 종류 분류 예측
            type_result = predict_type(image_bytes)
            await websocket.send(json.dumps(type_result))

        except Exception as e:
            await websocket.send(json.dumps({"error": str(e)}))


async def main():
    print("📡 WebSocket server started")
    async with websockets.serve(handle_client, "0.0.0.0", 9000):
        await asyncio.Future()  # Run forever

def start_socket_server():
    asyncio.run(main())