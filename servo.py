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

try:
    while True:
        # Move to the maximum position (typically 180 degrees)
        print("Moving to max position...")
        servo.max()
        sleep(1) # Wait for 1 second

        # Move to the mid position (typically 90 degrees)
        print("Moving to mid position...")
        servo.mid()
        sleep(1) # Wait for 1 second

        # Move to the minimum position (typically 0 degrees)
        print("Moving to min position...")
        servo.min()
        sleep(1) # Wait for 1 second

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    # Cleanup GPIO settings
    servo.close()
    print("GPIO cleanup complete.")