from gpiozero import DigitalInputDevice
from time import sleep

# --- CONFIGURATION ---
# IMPORTANT: Replace 27 with the actual GPIO pin number where your IR sensor's signal pin is connected.
IR_PROXIMITY_PIN = 27 

# Initialize the IR sensor as a digital input device
# Set pull_up=True if your sensor output is HIGH (3.3V) when nothing is detected 
# and goes LOW (0V) when an object is detected. 
# If your sensor outputs LOW when nothing is detected, use pull_up=False (default).
# We'll use the default assumption for now.
ir_sensor = DigitalInputDevice(IR_PROXIMITY_PIN) 

# --- MAIN TEST LOOP ---
print("--- IR Proximity Sensor Test Started ---")
print(f"Reading GPIO Pin: {IR_PROXIMITY_PIN}")
print("Place an object in front of the sensor to see the status change.")
print("Press Ctrl+C to stop the test.")

try:
    while True:
        # Check the value of the sensor. 
        # .is_active returns True if the sensor is triggered (object detected).
        if ir_sensor.is_active:
            print(f"[ACTIVE] ✅ BEAN DETECTED! The sensor is triggered. (Value: {ir_sensor.value})")
        else:
            print(f"[IDLE] 🫘 No bean detected. (Value: {ir_sensor.value})")
        
        # Wait a short period before checking again
        sleep(0.1) 

except KeyboardInterrupt:
    print("\n--- Test Stopped by User ---")
finally:
    # Cleanup is usually not necessary for DigitalInputDevice, but it's good practice
    pass