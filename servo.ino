#include <Servo.h>

Servo servo1;   // first servo
Servo servo2;   // second servo

// Sequence for servo2
int stepIndex = 0;
int positions[5] = {30, 60, 100, 140, 160};

void setup() {
  Serial.begin(9600);

  servo1.attach(9);  
  servo2.attach(10);

  Serial.println("Ready. Commands:");
  Serial.println("servo1: LEFT / MID / RIGHT");
  Serial.println("servo2: next");
}

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // ------- SERVO 1 COMMANDS -------
    if (cmd == "LEFT") {
      Serial.println("Servo1 → LEFT");
      servo1.write(0);      
    }
    else if (cmd == "MID") {
      Serial.println("Servo1 → MID");
      servo1.write(90);     
    }
    else if (cmd == "RIGHT") {
      Serial.println("Servo1 → RIGHT");
      servo1.write(180);    
    }

    // ------- SERVO 2 COMMAND -------
    else if (cmd == "NEXT") {
      moveServo2Steps();
    }
  }
}

void moveServo2Steps() {

  if (stepIndex < 5) {
    int pos = positions[stepIndex];

    Serial.print("Servo2 move to: ");
    Serial.println(pos);

    servo2.write(pos);
    stepIndex++;
  }

  if (stepIndex == 5) {
    Serial.println("Servo2 sequence done → resetting to 0");
    servo2.write(0);
    stepIndex = 0;
  }
}
