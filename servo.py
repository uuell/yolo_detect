from gpiozero import Servo
from time import sleep

# Define the BCM pin number your servo's signal wire is connected to
# We will use GPIO 17 (Physical Pin 11) as an example.
# **IMPORTANT:** Change this to the GPIO pin you actually use.
SERVO_PIN = 17

# Initialize the Servo object. 
# The servo object controls the PWM signal on the specified pin.
# We set the min_pulse_width and max_pulse_width for common 9g servos (adjust if needed)
servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

print(f"Servo initialized on GPIO {SERVO_PIN}. Press Ctrl+C to stop.")

# --- INITIAL NEUTRAL MOVE ---
print("Setting servo to NEUTRAL (Center, typically 90 degrees)...")
servo.mid() # Sets the servo value to 0.0 (neutral)
sleep(2) # Wait 2 seconds to ensure it reaches the starting position
print("Starting sweep sequence...")
# ----------------------------

try:
    while True:
        # 1. Move to the LEFT position (minimum)
        # Value = -1.0, typically 0 degrees
        print("Moving to LEFT (Minimum position, typically 0 degrees)...")
        servo.min() 
        sleep(1.5) # Wait for 1.5 seconds

        # 2. Move back to the NEUTRAL position (center)
        # Value = 0.0, typically 90 degrees
        print("Moving to NEUTRAL (Center position, typically 90 degrees)...")
        servo.mid()
        sleep(1.5) # Wait for 1.5 seconds

        # 3. Move to the RIGHT position (maximum)
        # Value = +1.0, typically 180 degrees
        print("Moving to RIGHT (Maximum position, typically 180 degrees)...")
        servo.max()
        sleep(1.5) # Wait for 1.5 seconds

        # 4. Move back to the NEUTRAL position (center)
        # Value = 0.0, typically 90 degrees
        print("Moving to NEUTRAL (Center position, typically 90 degrees)...")
        servo.mid()
        sleep(1.5) # Wait for 1.5 seconds


except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    # Cleanup GPIO settings
    servo.close()
    print("GPIO cleanup complete.")