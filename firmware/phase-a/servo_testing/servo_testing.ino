#include <Servo.h>

// SG90 on L293D shield SERV1 header (Arduino D10)
Servo servo1;
const int SERVO_PIN = 10;

// Positions (adjust if your horn alignment differs)
const int SERVO_CENTER = 90;     // neutral
const int SERVO_PLUS60 = 150;    // ~+60° from center (clockwise on most setups)

void setup() {
  servo1.attach(SERVO_PIN);
  servo1.write(SERVO_CENTER);
  delay(300);
}

void loop() {
  // Move +60° from center
  servo1.write(SERVO_PLUS60);
  delay(1000);

  // Return to center
  servo1.write(SERVO_CENTER);
  delay(1000);
}