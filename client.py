import cv2
import asyncio
import websockets
import base64
from gpiozero import Servo
import signal
import sys

SERVO_PIN = 17
servo = Servo(SERVO_PIN)

SERVER_URL = "ws://YOUR_SERVER_IP:8765"

# --- Safe shutdown handler ---
def safe_exit(*args):
    print("\n[❌] Exiting safely...")
    servo.value = 0
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, safe_exit)  # CTRL + C handler

# --- Servo Movement Helper ---
def move_servo(direction):
    if direction == "MOLD":
        servo.value = 1
        print("[⚠️] Mold detected → Servo RIGHT")
    elif direction == "GOOD":
        servo.value = -1
        print("[✅] Good bean detected → Servo LEFT")
    else:
        print("[ℹ️] No detection")

    asyncio.create_task(reset_servo())

async def reset_servo():
    await asyncio.sleep(0.4)
    servo.value = 0

# --- Main WebSocket Loop ---
async def send_frames():
    print("[🔄] Connecting to server...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[❌] ERROR: Camera failed to open.")
        safe_exit()

    try:
        async with websockets.connect(SERVER_URL) as ws:
            print("[✅] Connected to server!")

            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[❌] ERROR: Failed to capture frame.")
                    await asyncio.sleep(0.1)
                    continue

                # ✅ Show camera POV
                cv2.imshow("Camera POV", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    safe_exit()

                try:
                    # Convert frame to base64
                    _, buffer = cv2.imencode('.jpg', frame)
                    jpg_as_text = base64.b64encode(buffer).decode()
                except Exception as e:
                    print(f"[❌] ERROR encoding frame: {e}")
                    continue

                try:
                    await ws.send(jpg_as_text)
                except websockets.ConnectionClosed:
                    print("[❌] Server disconnected.")
                    break

                try:
                    result = await ws.recv()
                except websockets.ConnectionClosed:
                    print("[❌] Lost connection to server.")
                    break

                print(f"[📩] Server result: {result}")
                move_servo(result)

    except Exception as e:
        print(f"[❌] ERROR connecting to server: {e}")

    finally:
        print("[🔁] Attempting reconnect in 2 seconds...")
        await asyncio.sleep(2)
        await send_frames()   # reconnect automatically

asyncio.run(send_frames())
