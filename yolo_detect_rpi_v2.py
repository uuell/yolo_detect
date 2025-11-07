import os
import sys
import argparse
import glob
import time

import cv2
import numpy as np
from ultralytics import YOLO
from gpiozero import Servo, DigitalInputDevice # Import DigitalInputDevice for the IR Sensor
from RPLCD.i2c import CharLCD

import serial
import serial.tools.list_ports

# Define and parse user input arguments
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

# Function to send commands to the Arduino
def send_command(ser, command):
    ser.write(command.encode())  # Send the command to Arduino
    print(f"Sent command: {command}")

# Find Arduino and get the serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)  # Change to /dev/ttyACM0 for your Arduino

# I2C address
lcd = CharLCD('PCF8574', 0x27)

# NEW: IR Proximity Sensor Pin Setup
# CHOOSE A FREE GPIO PIN FOR THE IR SENSOR (e.g., GPIO 27)
IR_PROXIMITY_PIN = 27 
ir_sensor = DigitalInputDevice(IR_PROXIMITY_PIN) 
# Note: For many IR sensors, is_active (True) means an object IS present. 
# Check your sensor's documentation to see if it outputs HIGH or LOW on detection.

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
    camera = cap

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
last_detection_time = 0
DETECTION_INTERVAL = 15 
object_count = 0 # Moved outside loop so it persists when skipping frames
good_beans_count = 0
bad_beans_count = 0

# Begin inference loop
while True:

    t_start = time.perf_counter()

    # Load frame from image source
    if source_type == 'video' or source_type == 'usb':
        ret, frame = cap.read()
        if not ret: 
            # Handle end of file or camera disconnect
            break
    elif source_type == 'picamera':
        frame = cap.capture_array()
        if frame is None:
             break
    elif source_type == 'image':
        frame = cv2.imread(imgs_list[img_count])
        img_count += 1
        if img_count >= len(imgs_list):
            break
    elif source_type == 'folder':
        if img_count >= len(imgs_list):
            break
        frame = cv2.imread(imgs_list[img_count])
        img_count += 1
    
    # Resize frame to desired display resolution
    if resize == True:
        frame = cv2.resize(frame,(resW,resH))

    # --- IR PROXIMITY GATE (FIXED) ---
    # Check if a bean is present in the detection area
    #bean_not_present = ir_sensor.is_active  # FIXED: Renamed variable to reflect actual meaning
    
    #run_inference = False # Default to not running inference

    #if not bean_not_present:  # FIXED: Added 'not' here
         # If a bean is present, proceed with the frame skipping logic
        #FRAME_COUNTER += 1
        #if (FRAME_COUNTER % FRAME_SKIP == 0):
            #run_inference = True
            #FRAME_COUNTER = 0
   # else:
         # No bean present, reset the frame counter and keep servo neutral
       #FRAME_COUNTER = 0
        # set the servo to neutral
        #send_command(ser, "MID")
       # print("set servo to neutral")
    # --- END IR PROXIMITY GATE ---

    if source_type == 'video' or source_type == 'usb':
        for _ in range(5):
            cap.grab()
            ret, frame = cap.read()
            if not ret:
                break
    elif source_type == 'picamera':
        frame = cap.capture_array()
        if frame is None:
            break_anywhere
    
    current_time = time.perf_counter()
    run_inference = False
    
    if(current_time - last_detection_time) >= DETECTION_INTERVAL:
        run_inference = True
    
    # ----------------------------------------------------
    # --- YOLO INFERENCE AND SERVO CONTROL BLOCK ---
    # ----------------------------------------------------
    if run_inference:
        
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
                if classname == 'coffee-beans':
                    good_bean_detected_in_frame = True
                    print("good bean detected")
                    good_beans_count += 1
                    lcd.clear()
                    lcd.write_string(f"Good Beans: {good_beans_count}")
                    lcd.crlf()
                    lcd.write_string(f"Bad Beans: {bad_beans_count}")
                    send_command(ser, "LEFT")
                    time.sleep(6)
                    send_command(ser, "MID")
                    time.sleep(6)
                    last_detection_time = time.perf_counter()
                elif classname == 'coffee-bean mold':
                    mold_detected_in_frame = True
                    print("bad bean detected")
                    bad_beans_count += 1
                    lcd.clear()
                    lcd.write_string(f"Good Beans: {good_beans_count}")
                    lcd.crlf()
                    lcd.write_string(f"Bad Beans: {bad_beans_count}")
                    send_command(ser, "RIGHT")
                    time.sleep(6)
                    send_command(ser, "MID")
                    time.sleep(6)
                    last_detection_time = time.perf_counter()  
                
                # Drawing and labeling logic
                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)
                
                # Draw label
                label_text = f'{classname}: {conf:.2f}'
                label_size, baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                label_ymin = max(ymin, label_size[1] + 10)
                cv2.rectangle(frame, (xmin, label_ymin-label_size[1]-10), (xmin+label_size[0], label_ymin+baseline-10), color, cv2.FILLED)
                cv2.putText(frame, label_text, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

                object_count += 1

    # --- END YOLO INFERENCE BLOCK ---
    # ----------------------------------------------------

    # Calculate and draw framerate
    if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) 
    
    # Display detection results
    # NOTE: object_count displays the count from the last frame where inference ran.
    cv2.putText(frame, f'Beans Detected (Last Inference): {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
    cv2.putText(frame, f'Good Beans Detected (Last Inference): {good_beans_count}', (10,60), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) 
    cv2.putText(frame, f'Bad Beans Detected (Last Inference): {bad_beans_count}', (10,80), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) 
     
    # FIXED: Show correct sensor status
    #cv2.putText(frame, f'Bean Present: {bean_not_present}', (10,100), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
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
