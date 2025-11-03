from gpiozero import DigitalInputDevice, Servo
from time import sleep

# --- HARDWARE CONFIGURATION ---

# 1. IR Proximity Sensor Pin
# IMPORTANT: Connect the IR sensor's signal pin to this GPIO pin.
IR_PROXIMITY_PIN = 27 
ir_sensor = DigitalInputDevice(IR_PROXIMITY_PIN) 
# Note: DigitalInputDevice assumes the pin is HIGH when inactive.
# If your sensor is LOW when inactive, you may need: ir_sensor = DigitalInputDevice(IR_PROXIMITY_PIN, pull_up=True)

# 2. Servo Pin Configuration
# Connect the servo's signal pin to this GPIO pin.
SERVO_PIN = 17
servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

# --- SERVO ROTATION VALUES (For a Continuous Rotation Servo) ---
# Assuming a continuous rotation servo:
SERVO_STOP = 0.0          # Neutral/Stop position (0.0 is often the neutral point for gpiozero's Servo)
SERVO_LEFT = -1.0         # Full speed Counter-Clockwise (CCW)
SERVO_RIGHT = 1.0         # Full speed Clockwise (CW)

# --- MAIN TEST LOOP ---
print("--- IR Sensor & Servo Test Started ---")
print(f"IR Sensor on GPIO: {IR_PROXIMITY_PIN} | Servo on GPIO: {SERVO_PIN}")
print("Place an object in front of the IR sensor to trigger the servo movement.")
print("Press Ctrl+C to stop the test.")

# Ensure the servo is in the neutral position at the start
servo.value = SERVO_STOP 

try:
    while True:
        # Check the sensor state
        if ir_sensor.is_active:
            print("\n[ACTIVE] ✅ BEAN DETECTED! Starting sorting sequence...")
            
            # 1. Rotate Left (Sweep)
            print("         > Rotating LEFT...")
            servo.value = SERVO_LEFT
            sleep(0.5) # Time for the sweep action
            
            # 2. Rotate Right (Sweep)
            print("         > Rotating RIGHT...")
            servo.value = SERVO_RIGHT
            sleep(0.5) # Time for the sweep action
            
            # 3. Return to Neutral
            print("         > Returning to NEUTRAL/STOP.")
            servo.value = SERVO_STOP
            
            # Wait a moment before checking for the next bean
            sleep(1.0) 

        else:
            # If no bean is detected, keep the servo stationary
            print("[IDLE] No bean detected.")
            servo.value = SERVO_STOP
            sleep(0.1) 

except KeyboardInterrupt:
    print("\n--- Test Stopped by User ---")
finally:
    # Always ensure the servo is fully turned off on exit
    servo.close() 
    print("Servo released.")