import cv2
import asyncio
import websockets
import base64
import json
from gpiozero import Servo

SERVO_PIN = 17
servo = Servo(SERVO_PIN)

async def send_frames():
    async with websockets.connect("ws://YOUR_SERVER_IP:8765") as ws:
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Encode frame to base64
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode()

            # Send frame
            await ws.send(jpg_as_text)

            # Receive YOLO result
            result = await ws.recv()
            print("Server result:", result)

            # Servo control
            if result == "MOLD":
                servo.value = 1     # go to mold side
                print("mold detected")
            elif result == "GOOD":
                servo.value = -1    # go to good side
                print("good bean detected")

            # Return to neutral
            await asyncio.sleep(0.5)
            servo.value = 0

asyncio.run(send_frames())
