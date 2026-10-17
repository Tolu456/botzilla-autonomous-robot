#include <AFMotor.h>
#include <Wire.h>
#include <VL53L0X.h>

// ------------ Motors (Adafruit L293D shield v1 - AFMotor) ------------
#define BASE_SPEED 100          // keep <= 90 for reliable spin-up
#define BIAS_DELTA 35          // slow LEFT side by this much to create a left arc
#define CORRECTION_DELTA 20    // wall-follow trim
#define MAX_SPEED_LIMIT 150     // hard cap due to power constraints

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Flip these if forward motion is reversed due to wiring
const uint8_t LEFT_FWD  = FORWARD;   // change to BACKWARD if needed
const uint8_t RIGHT_FWD = FORWARD;   // change to BACKWARD if needed

// Soft-start ramp
uint8_t currentLeftSpeed = 0;
uint8_t currentRightSpeed = 0;
const uint8_t RAMP_STEP = 3;             // smaller = gentler ramp
const uint16_t RAMP_INTERVAL_MS = 20;

static inline uint8_t clampSpeedInt(int v) {
  if (v < 0) return 0;
  if (v > MAX_SPEED_LIMIT) return (uint8_t)MAX_SPEED_LIMIT;
  return (uint8_t)v;
}

static inline uint8_t stepToward(uint8_t current, uint8_t target, uint8_t step) {
  if (current < target) {
    int next = (int)current + (int)step;
    if (next > target) next = target;
    return (uint8_t)next;
  } else if (current > target) {
    int next = (int)current - (int)step;
    if (next < target) next = target;
    return (uint8_t)next;
  } else {
    return current;
  }
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void forward() {
  motor1.run(LEFT_FWD);
  motor2.run(LEFT_FWD);
  motor3.run(RIGHT_FWD);
  motor4.run(RIGHT_FWD);
}

void applyRampedSpeeds(uint8_t targetLeft, uint8_t targetRight) {
  currentLeftSpeed = stepToward(currentLeftSpeed, targetLeft, RAMP_STEP);
  currentRightSpeed = stepToward(currentRightSpeed, targetRight, RAMP_STEP);

  motor1.setSpeed(currentLeftSpeed);
  motor2.setSpeed(currentLeftSpeed);
  motor3.setSpeed(currentRightSpeed);
  motor4.setSpeed(currentRightSpeed);
}

// ------------ VL53L0X (3 sensors) ------------
VL53L0X sensorRightFront;
VL53L0X sensorRightRear;
VL53L0X sensorCenter;

// XSHUT pins (use analogs as digital)
const uint8_t SensorRightFrontXshut = A0;
const uint8_t SensorRightRearXshut  = A1;
const uint8_t SensorCenterXshut     = A2;

// I2C addresses
const uint8_t SensorRightFrontAddress = 0x30;
const uint8_t SensorRightRearAddress  = 0x31;
const uint8_t SensorCenterAddress     = 0x32;

// Offsets and thresholds
const int CM_OFFSET = 2;           // +2 cm to all three sensors
const int STOP_DISTANCE_CM = 20;   // stop when center <= 20 cm
const int PARALLEL_TOLERANCE = 2;  // cm band for right-front vs right-rear

int readCmWithOffset(VL53L0X& s) {
  int mm = s.readRangeContinuousMillimeters();
  if (s.timeoutOccurred()) return 999; // treat timeout as very far
  int cm = (mm / 10) + CM_OFFSET;
  if (cm < 0) cm = 0;
  return cm;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Motors off during sensor bring-up
  stopAll();
  currentLeftSpeed = 0;
  currentRightSpeed = 0;

  // XSHUT control pins
  pinMode(SensorRightFrontXshut, OUTPUT);
  pinMode(SensorRightRearXshut, OUTPUT);
  pinMode(SensorCenterXshut, OUTPUT);

  // Hold all sensors in reset
  digitalWrite(SensorRightFrontXshut, LOW);
  digitalWrite(SensorRightRearXshut, LOW);
  digitalWrite(SensorCenterXshut, LOW);
  delay(10);

  // Bring up right-front
  digitalWrite(SensorRightFrontXshut, HIGH);
  delay(10);
  if (!sensorRightFront.init()) Serial.println("Right Front init FAILED");
  else { sensorRightFront.setAddress(SensorRightFrontAddress); Serial.println("Right Front OK @0x30"); }

  // Bring up right-rear
  digitalWrite(SensorRightRearXshut, HIGH);
  delay(10);
  if (!sensorRightRear.init()) Serial.println("Right Rear init FAILED");
  else { sensorRightRear.setAddress(SensorRightRearAddress); Serial.println("Right Rear OK @0x31"); }

  // Bring up center
  digitalWrite(SensorCenterXshut, HIGH);
  delay(10);
  if (!sensorCenter.init()) Serial.println("Center init FAILED");
  else { sensorCenter.setAddress(SensorCenterAddress); Serial.println("Center OK @0x32"); }

  // Continuous mode
  sensorRightFront.startContinuous();
  sensorRightRear.startContinuous();
  sensorCenter.startContinuous();

  // Stagger motor spin-up to reduce surge
  motor1.run(RELEASE); motor2.run(RELEASE); motor3.run(RELEASE); motor4.run(RELEASE);
  delay(50);
  motor1.run(LEFT_FWD); motor2.run(LEFT_FWD);
  delay(80);
  motor3.run(RIGHT_FWD); motor4.run(RIGHT_FWD);

  delay(100);
}

void loop() {
  int rightFront = readCmWithOffset(sensorRightFront);
  int rightRear  = readCmWithOffset(sensorRightRear);
  int center     = readCmWithOffset(sensorCenter);

  // Stop when center <= 20 cm
  if (center <= STOP_DISTANCE_CM) {
    stopAll();
    Serial.println("Stop: center <= 20 cm");
    while (1) { delay(100); }
  }

  // LEFT ARC bias: slow LEFT side to create a left curve
  int leftTarget  = BASE_SPEED;
  int rightTarget = BASE_SPEED - BIAS_DELTA;

  // Wall-follow trim around the bias using right-front vs right-rear
  int diff = rightFront - rightRear;
  if (diff > PARALLEL_TOLERANCE) {
    // nose away from wall -> turn right slightly: slow LEFT more
    leftTarget -= CORRECTION_DELTA;
  } else if (diff < -PARALLEL_TOLERANCE) {
    // nose toward wall -> turn left slightly: slow RIGHT a bit
    rightTarget -= CORRECTION_DELTA;
  }

  // Clamp to safe max and apply ramp
  uint8_t finalLeft  = clampSpeedInt(leftTarget);
  uint8_t finalRight = clampSpeedInt(rightTarget);
  applyRampedSpeeds(finalLeft, finalRight);
  forward();

  uint16_t loopDelay = (RAMP_INTERVAL_MS > 50) ? RAMP_INTERVAL_MS : 50;
  delay(loopDelay);
}