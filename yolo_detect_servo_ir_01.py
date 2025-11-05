import os
import sys
import argparse
import glob
import time

import cv2
import numpy as np
from ultralytics import YOLO
from gpiozero import Servo, DigitalInputDevice # Import DigitalInputDevice for the IR Sensor
from time import sleep

# Define and parse user input arguments
# (omitted for brevity, assume the original argparse block is here)
# ...
parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                     required=True)
parser.add_argument('--source', help='Image source, can be image file ("test.jpg"), \
                     image folder ("test_dir"), video file ("testvid.mp4"), index of USB camera ("usb0"), or index of Picamera ("picamera0")', 
                     required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                     default=0.5)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                     otherwise, match source resolution',
                     default=None)
parser.add_argument('--record', help='Record results from video or webcam and save it as "demo1.avi". Must specify --resolution argument to record.',
                     action='store_true')

args = parser.parse_args()


# Parse user inputs
model_path = args.model
img_source = args.source
min_thresh = args.thresh
user_res = args.resolution
record = args.record

# --- HARDWARE PIN CONFIGURATION ---

# Servo Pin Setup
SERVO_PIN = 17
servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo_enabled = True

# NEW: IR Proximity Sensor Pin Setup
# CHOOSE A FREE GPIO PIN FOR THE IR SENSOR (e.g., GPIO 27)
IR_PROXIMITY_PIN = 27 
ir_sensor = DigitalInputDevice(IR_PROXIMITY_PIN) 
# Note: For many IR sensors, is_active (True) means an object IS present. 
# Check your sensor's documentation to see if it outputs HIGH or LOW on detection.

print(f"Servo Initialized on GPIO {SERVO_PIN}.")
print(f"Servo Initialized on GPIO {SERVO_PIN_2}.")
print(f"IR Proximity Sensor Initialized on GPIO {IR_PROXIMITY_PIN}.")

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

# Parse input to determine if image source is a file, folder, video, or USB camera
img_ext_list = ['.jpg','.JPG','.jpeg','.JPEG','.png','.PNG','.bmp','.BMP']
vid_ext_list = ['.avi','.mov','.mp4','.mkv','.wmv']

if os.path.isdir(img_source):
    source_type = 'folder'
elif os.path.isfile(img_source):
    _, ext = os.path.splitext(img_source)
    if ext in img_ext_list:
        source_type = 'image'
    elif ext in vid_ext_list:
        source_type = 'video'
    else:
        print(f'File extension {ext} is not supported.')
        sys.exit(0)
elif 'usb' in img_source:
    source_type = 'usb'
    usb_idx = int(img_source[3:])
elif 'picamera' in img_source:
    source_type = 'picamera'
    picam_idx = int(img_source[8:])
else:
    print(f'Input {img_source} is invalid. Please try again.')
    sys.exit(0)

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Check if recording is valid and set up recording
if record:
    if source_type not in ['video','usb']:
        print('Recording only works for video and camera sources. Please try again.')
        sys.exit(0)
    if not user_res:
        print('Please specify resolution to record video at.')
        sys.exit(0)
    
    # Set up recording
    record_name = 'demo1.avi'
    record_fps = 30
    recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))

# Load or initialize image source
if source_type == 'image':
    imgs_list = [img_source]
elif source_type == 'folder':
    imgs_list = []
    filelist = glob.glob(img_source + '/*')
    for file in filelist:
        _, file_ext = os.path.splitext(file)
        if file_ext in img_ext_list:
            imgs_list.append(file)
elif source_type == 'video' or source_type == 'usb':

    if source_type == 'video': cap_arg = img_source
    elif source_type == 'usb': cap_arg = usb_idx
    cap = cv2.VideoCapture(cap_arg)

    # Set camera or video resolution if specified by user
    if user_res:
        ret = cap.set(3, resW)
        ret = cap.set(4, resH)

elif source_type == 'picamera':
    from picamera2 import Picamera2
    cap = Picamera2()
    cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
    cap.start()

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
               (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

# Frame Skipping Variables
FRAME_COUNTER = 0
FRAME_SKIP = 12 
object_count = 0 # Moved outside loop so it persists when skipping frames
good_beans_count = 0
bad_beans_count = 0

 # Servo position settings (Should be defined outside loop, but keeping here for context)
servo_position_good = -1.0
servo_position_mold = 1.0
servo_position_neutral = 0.0

# Begin inference loop
while True:

    t_start = time.perf_counter()

    # Load frame from image source (Same as original code)
    # ... (Loading frame logic for image, folder, video, usb, picamera) ...
    # Ensure 'frame' variable is loaded correctly from your source type.

    # Example Frame Loading (Using USB/Video as a placeholder)
    if source_type == 'video' or source_type == 'usb':
        ret, frame = cap.read()
        if not ret: 
            # Handle end of file or camera disconnect
            break
    elif source_type == 'picamera':
        frame = cap.capture_array()
        if frame is None:
             break
    # ... (Other source type handlers) ...
    
    # Resize frame to desired display resolution
    if resize == True:
        frame = cv2.resize(frame,(resW,resH))

    # --- IR PROXIMITY GATE (NEW) ---
    # Check if a bean is present in the detection area
    bean_not_present = ir_sensor.is_active 
    
    run_inference = False # Default to not running inference

    if bean_not_present:
        # No bean present, reset the frame counter and keep servo neutral
        FRAME_COUNTER = 0
        if servo_enabled:
             # Ensure the servo is neutral/closed when no bean is present
             servo.value = 0.0 
       
    else:
        # If a bean is present, proceed with the frame skipping logic
        FRAME_COUNTER += 1
        if (FRAME_COUNTER % FRAME_SKIP == 0):
            run_inference = True
            FRAME_COUNTER = 0
    # --- END IR PROXIMITY GATE ---

    
    # ----------------------------------------------------
    # --- YOLO INFERENCE AND SERVO CONTROL BLOCK ---
    # ----------------------------------------------------
    if run_inference:
        time.sleep(2)
        
        
        # Run inference on frame
        results = model(frame, verbose=False)

        # Extract results
        detections = results[0].boxes

        # Initialize per-inference status
        object_count = 0
        good_bean_detected_in_frame = False
        mold_detected_in_frame = False

        # Go through each detection
        for i in range(len(detections)):

            # Get bounding box coordinates and class info
            xyxy_tensor = detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int) 
            classidx = int(detections[i].cls.item())
            classname = labels[classidx]
            conf = detections[i].conf.item()

            # Draw box and update status if confidence threshold is high enough
            if conf > float(min_thresh):
                print(classname)
                mold_detected_in_frame = True
                bad_beans_count += 1
                print("bad bean detected")

                # Drawing and labeling logic (Same as original code)
                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)
                # ... (rest of drawing/labeling code) ...

                object_count += 1
        
        ### Servo Logic (ONLY runs when inference is run)
        if servo_enabled:
            if mold_detected_in_frame:
                servo.value = servo_position_mold # Push to mold chute
                time.sleep(1.5)
                servo.value = servo_position_neutral
            elif good_bean_detected_in_frame:
                # If only good beans are detected, let it pass (servo neutral) or push to good chute
                # Assuming the bean should be sorted if detected. 
                # If a good bean is detected, you might want to move it too, or just let it pass.
                servo.value = servo_position_good # Example: Push to good chute
                time.sleep(1.5)
                servo.value = servo_position_neutral
            else:
                # No relevant object detected, keep servo neutral.
                servo.value = servo_position_neutral

    # --- END YOLO INFERENCE BLOCK ---
    # ----------------------------------------------------

    # Calculate and draw framerate (Same as original code)
    if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) 
    
    # Display detection results
    # NOTE: object_count displays the count from the last frame where inference ran.
    cv2.putText(frame, f'Beans Detected (Last Inference): {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
    cv2.putText(frame, f'Good Beans Detected (Last Inference): {good_beans_count}', (10,60), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) 
    cv2.putText(frame, f'Bad Beans Detected (Last Inference): {bad_beans_count}', (10,80), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) 
     
    # NEW: Show sensor status
    cv2.putText(frame, f'IR Active: {bean_not_present}', (10,100), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
    cv2.imshow('YOLO detection results',frame)
    if record: recorder.write(frame)

    # If inferencing on individual images, wait for user keypress before moving to next image. Otherwise, wait 5ms before moving to next frame.
    if source_type == 'image' or source_type == 'folder':
        key = cv2.waitKey()
    elif source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        key = cv2.waitKey(5)
    
    if key == ord('q') or key == ord('Q'): 
        break
    elif key == ord('s') or key == ord('S'): 
        cv2.waitKey()
    elif key == ord('p') or key == ord('P'): 
        cv2.imwrite('capture.png',frame)
        
    # Calculate FPS for this frame
    t_stop = time.perf_counter()
    frame_rate_calc = float(1/(t_stop - t_start))

    # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
    if len(frame_rate_buffer) >= fps_avg_len:
        temp = frame_rate_buffer.pop(0)
        frame_rate_buffer.append(frame_rate_calc)
    else:
        frame_rate_buffer.append(frame_rate_calc)

    # Calculate average FPS for past frames
    avg_frame_rate = np.mean(frame_rate_buffer)

# Clean up
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
if source_type == 'video' or source_type == 'usb':
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record: recorder.release()
cv2.destroyAllWindows()
