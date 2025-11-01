import asyncio
import websockets
import base64
import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("best.pt")   # your yolo model

async def handler(websocket):
    print("Pi connected")

    while True:
        # Receive base64 frame from Pi
        data = await websocket.recv()
        
        # Convert base64 → image
        img_bytes = base64.b64decode(data)
        np_arr = np.frombuffer(img_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # --- Run YOLO here ---
        results = model(frame)[0]
        classes = [model.names[c] for c in results.boxes.cls]

        # Decide response
        if "coffee beans mold" in classes:
            await websocket.send("MOLD")
        elif "coffee beans" in classes:
            await websocket.send("GOOD")
        else:
            await websocket.send("NONE")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("Server listening on port 8765")
        await asyncio.Future()  # run forever

asyncio.run(main())