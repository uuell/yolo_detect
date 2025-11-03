import os
import sys
import argparse
import glob
import time

import cv2
import numpy as np
from ultralytics import YOLO

# ==============================
# 1. Servo Library Import
# ==============================
try:
    from gpiozero import Servo
    SERVO_PIN = 17  # Change to your actual GPIO pin
    servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
    SERVO_ENABLED = True
    print(f"✅ Servo initialized on GPIO {SERVO_PIN}.")
except ImportError:
    print("⚠️ gpiozero not found. Servo control will be skipped.")
    SERVO_ENABLED = False
except Exception as e:
    print(f"⚠️ Could not initialize servo: {e}. Servo control disabled.")
    SERVO_ENABLED = False


# ==============================
# 2. Argument Parser
# ==============================
parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True, help='Path to YOLO model (e.g. "runs/detect/train/weights/best.pt")')
parser.add_argument('--source', required=True, help='Image, folder, video file, "usb0", or "picamera0"')
parser.add_argument('--thresh', default=0.5, type=float, help='Confidence threshold (e.g. 0.4)')
parser.add_argument('--resolution', default=None, help='WxH resolution (e.g. "640x480")')
parser.add_argument('--record', action='store_true', help='Record output video (requires --resolution)')
args = parser.parse_args()


# ==============================
# 3. Load Model
# ==============================
model_path = args.model
img_source = args.source
min_thresh = args.thresh
user_res = args.resolution
record = args.record

if not os.path.exists(model_path):
    print('❌ ERROR: Model path invalid or not found.')
    sys.exit(0)

model = YOLO(model_path, task='detect')
labels = model.names

# ==============================
# 4. Determine Source Type
# ==============================
img_ext_list = ['.jpg', '.jpeg', '.png', '.bmp']
vid_ext_list = ['.avi', '.mov', '.mp4', '.mkv', '.wmv']

if os.path.isdir(img_source):
    source_type = 'folder'
elif os.path.isfile(img_source):
    _, ext = os.path.splitext(img_source)
    if ext in img_ext_list:
        source_type = 'image'
    elif ext in vid_ext_list:
        source_type = 'video'
    else:
        print(f'❌ Unsupported file extension: {ext}')
        sys.exit(0)
elif 'usb' in img_source:
    source_type = 'usb'
    usb_idx = int(img_source[3:])
elif 'picamera' in img_source:
    source_type = 'picamera'
    picam_idx = int(img_source[8:])
else:
    print(f'❌ Invalid source: {img_source}')
    sys.exit(0)

# ==============================
# 5. Resolution and Recording
# ==============================
resize = False
if user_res:
    resize = True
    resW, resH = map(int, user_res.split('x'))

if record:
    if source_type not in ['video', 'usb']:
        print('❌ Recording only works for video and camera.')
        sys.exit(0)
    if not user_res:
        print('❌ Please specify resolution to record.')
        sys.exit(0)
    recorder = cv2.VideoWriter('demo1.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (resW, resH))

# ==============================
# 6. Load Source
# ==============================
if source_type == 'image':
    imgs_list = [img_source]
elif source_type == 'folder':
    imgs_list = [f for f in glob.glob(img_source + '/*') if os.path.splitext(f)[1] in img_ext_list]
elif source_type == 'video' or source_type == 'usb':
    cap_arg = img_source if source_type == 'video' else usb_idx
    cap = cv2.VideoCapture(cap_arg)
    if user_res:
        cap.set(3, resW)
        cap.set(4, resH)
elif source_type == 'picamera':
    from picamera2 import Picamera2
    cap = Picamera2()
    cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
    cap.start()

# ==============================
# 7. Setup Display and Variables
# ==============================
bbox_colors = [
    (164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106),
    (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)
]

avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

coffee_beans_count = 0
mold_count = 0
object_count = 0

# Servo position settings
SERVO_POSITION_GOOD = -1.0
SERVO_POSITION_MOLD = 1.0
SERVO_POSITION_NEUTRAL = 0.0

if SERVO_ENABLED:
    servo.value = SERVO_POSITION_NEUTRAL

# ==============================
# 8. Main Detection Loop
# ==============================
while True:
    t_start = time.perf_counter()
    mold_detected_in_frame = False
    good_bean_detected_in_frame = False

    if source_type in ['image', 'folder']:
        if img_count >= len(imgs_list):
            print('✅ All images processed. Exiting.')
            break
        frame = cv2.imread(imgs_list[img_count])
        img_count += 1
    elif source_type in ['video', 'usb']:
        ret, frame = cap.read()
        if not ret:
            print('✅ End of video or camera error. Exiting.')
            break
    elif source_type == 'picamera':
        frame = cap.capture_array()
        if frame is None:
            print('⚠️ Unable to read frames from Picamera.')
            break

    if resize:
        frame = cv2.resize(frame, (resW, resH))

    results = model(frame, verbose=False)
    detections = results[0].boxes

    for i in range(len(detections)):
        xyxy = detections[i].xyxy.cpu().numpy().squeeze()
        xmin, ymin, xmax, ymax = xyxy.astype(int)
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]
        conf = detections[i].conf.item()

        if conf > float(min_thresh):
            if classname == 'coffee beans':
                good_bean_detected_in_frame = True
                coffee_beans_count += 1
            elif classname == 'coffee beans mold':
                mold_detected_in_frame = True
                mold_count += 1

            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            label = f'{classname}: {int(conf*100)}%'
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            object_count += 1

    # ==============================
    # Servo Logic
    # ==============================
    if SERVO_ENABLED:
        if mold_detected_in_frame:
            servo.value = SERVO_POSITION_MOLD
            cv2.putText(frame, 'SERVO: MOLD (DISCARD)', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,0,255), 2)
            time.sleep(1.5)
            servo.value = SERVO_POSITION_NEUTRAL
        elif good_bean_detected_in_frame:
            servo.value = SERVO_POSITION_GOOD
            cv2.putText(frame, 'SERVO: GOOD (KEEP)', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,0), 2)
            time.sleep(1.5)
            servo.value = SERVO_POSITION_NEUTRAL

    # ==============================
    # Display Info
    # ==============================
    cv2.putText(frame, f'Total Beans: {coffee_beans_count}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
    cv2.putText(frame, f'Mold Count: {mold_count}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,0,255), 2)
    cv2.putText(frame, f'Objects: {object_count}', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)

    if source_type in ['video', 'usb', 'picamera']:
        cv2.putText(frame, f'FPS: {avg_frame_rate:.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)

    cv2.imshow('YOLO detection results', frame)
    if record:
        recorder.write(frame)

    key = cv2.waitKey(5 if source_type in ['video', 'usb', 'picamera'] else 0)
    if key in [ord('q'), ord('Q')]:
        break
    elif key in [ord('s'), ord('S')]:
        cv2.waitKey()
    elif key in [ord('p'), ord('P')]:
        cv2.imwrite('capture.png', frame)

    t_stop = time.perf_counter()
    frame_rate_calc = 1 / (t_stop - t_start)
    if len(frame_rate_buffer) >= fps_avg_len:
        frame_rate_buffer.pop(0)
    frame_rate_buffer.append(frame_rate_calc)
    avg_frame_rate = np.mean(frame_rate_buffer)

# ==============================
# 9. Cleanup
# ==============================
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
if source_type in ['video', 'usb']:
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record:
    recorder.release()
cv2.destroyAllWindows()

if SERVO_ENABLED:
    servo.close()
    print("✅ Servo resources released.")
