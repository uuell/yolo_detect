#include <Servo.h>

Servo myServo;
Servo myServo2;

int SERVO_PIN = 9;
int SERVO_PIN_2 = 10;

String command = "";

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  myServo2.attach(SERVO_PIN_2);

  Serial.println("Servo Control Ready.");
  Serial.println("Commands:");
  Serial.println("  LEFT / MID / RIGHT  -> Servo 1");
  Serial.println("  ZERO to FOUR        -> Servo 2");
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();

    // ==== SERVO 1 CONTROLS ====
    if (command.equalsIgnoreCase("LEFT")) {
      Serial.println("Servo1 -> LEFT (0 degrees)");
      myServo.write(0);
    } 
    else if (command.equalsIgnoreCase("MID")) {
      Serial.println("Servo1 -> MID (90 degrees)");
      myServo.write(90);
    }
    else if (command.equalsIgnoreCase("RIGHT")) {
      Serial.println("Servo1 -> RIGHT (180 degrees)");
      myServo.write(180);
    }

    // ==== SERVO 2 CONTROLS ====
    else if (command.equalsIgnoreCase("ZERO")) {
      Serial.println("Servo2 -> 0 degrees");
      myServo2.write(0);
    }
    else if (command.equalsIgnoreCase("ONE")) {
      Serial.println("Servo2 -> 40 degrees");
      myServo2.write(40);
    }
    else if (command.equalsIgnoreCase("TWO")) {
      Serial.println("Servo2 -> 80 degrees");
      myServo2.write(80);
    }
    else if (command.equalsIgnoreCase("THREE")) {
      Serial.println("Servo2 -> 120 degrees");
      myServo2.write(120);
    }
    else if (command.equalsIgnoreCase("FOUR")) {
      Serial.println("Servo2 -> 180 degrees");
      myServo2.write(180);
    }

    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }
}
