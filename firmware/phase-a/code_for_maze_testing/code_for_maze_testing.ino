#include <AFMotor.h>
#include <Wire.h>
#include <VL53L0X.h>

AF_DCMotor L1(1), L2(2), R1(3), R2(4);
VL53L0X front;

void setup() {
  Wire.begin();
  front.init();
  front.startContinuous();

  // start driving
  L1.run(FORWARD); L2.run(FORWARD); R1.run(FORWARD); R2.run(FORWARD);
  L1.setSpeed(150); L2.setSpeed(150); R1.setSpeed(150); R2.setSpeed(150);

  // keep moving until wall ≤15 cm
  while (front.readRangeContinuousMillimeters()/10 > 15) {}

  // stop
  L1.run(RELEASE); L2.run(RELEASE); R1.run(RELEASE); R2.run(RELEASE);
  while(1); // done
}

void loop() {}