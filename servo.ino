#include <Servo.h>

Servo myServo;

// Change to your servo signal pin
int SERVO_PIN = 9;

// Current command string
String command = "";

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);

  Serial.println("Servo Control Ready.");
  Serial.println("Commands:");
  Serial.println("  LEFT  -> Move to 0 degrees");
  Serial.println("  MID   -> Move to 90 degrees");
  Serial.println("  RIGHT -> Move to 180 degrees");
}

void loop() {
  // Check if something is received
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();  // Remove spaces & newline

    // Process commands
    if (command.equalsIgnoreCase("LEFT")) {
      Serial.println("Moving to LEFT (0 degrees)...");
      myServo.write(0);
    } 
    else if (command.equalsIgnoreCase("MID")) {
      Serial.println("Moving to MID (90 degrees)...");
      myServo.write(90);
    }
    else if (command.equalsIgnoreCase("RIGHT")) {
      Serial.println("Moving to RIGHT (180 degrees)...");
      myServo.write(180);
    } 
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }
}
