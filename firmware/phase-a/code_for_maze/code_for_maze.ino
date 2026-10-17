#include <AFMotor.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

#define MAX_SPEED 120
#define MAX_SPEED_OFFSET 20
#define WALL_FOLLOW_TOLERANCE 2  
#define TURN_DELAY 300          
#define FRONT_WALL_DISTANCE 20  
#define ANGLE_OFFSET 3

// Create VL53L0X objects
VL53L0X sensorLeftFront;
VL53L0X sensorLeftRear;
VL53L0X sensorCenter;

// XSHUT pins using analog pins as digital
const uint8_t SensorLeftFrontXshut = A2;
const uint8_t SensorLeftRearXshut = A0;
const uint8_t SensorCenterXshut = A1;

// I2C addresses for sensors
const uint8_t SensorLeftFrontAddress = 0x30;
const uint8_t SensorLeftRearAddress = 0x31;
const uint8_t SensorCenterAddress = 0x32;

AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;   

boolean goesForward = false;
int speedSet = 0;
int initialWallDistance = 0;
boolean isInitialized = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupToFSensors();
  myservo.attach(10);  
  myservo.write(115); 
  delay(2000);
}

void setupToFSensors() {
  pinMode(SensorLeftFrontXshut, OUTPUT);
  pinMode(SensorLeftRearXshut, OUTPUT);
  pinMode(SensorCenterXshut, OUTPUT);
  
  digitalWrite(SensorLeftFrontXshut, LOW);
  digitalWrite(SensorLeftRearXshut, LOW);
  digitalWrite(SensorCenterXshut, LOW);
  delay(10);
  
  digitalWrite(SensorLeftFrontXshut, HIGH);
  delay(10);
  if (!sensorLeftFront.init()) {
    Serial.println("Left Front sensor failed");
  }
  sensorLeftFront.setAddress(SensorLeftFrontAddress);
  
  digitalWrite(SensorLeftRearXshut, HIGH);
  delay(10);
  if (!sensorLeftRear.init()) {
    Serial.println("Left Rear sensor failed");
  }
  sensorLeftRear.setAddress(SensorLeftRearAddress);
  
  digitalWrite(SensorCenterXshut, HIGH);
  delay(10);
  if (!sensorCenter.init()) {
    Serial.println("Center sensor failed");
  }
  sensorCenter.setAddress(SensorCenterAddress);
  
  sensorLeftFront.startContinuous();
  sensorLeftRear.startContinuous();
  sensorCenter.startContinuous();
}

void loop() {
  int leftFront = max(0, (sensorLeftFront.readRangeContinuousMillimeters() / 10) - 3);
  int leftRear = max(0, (sensorLeftRear.readRangeContinuousMillimeters() / 10) - 3);
  int center = max(0, (sensorCenter.readRangeContinuousMillimeters() / 10) - 3);
  
  if (!isInitialized) {
    initialWallDistance = leftFront;
    isInitialized = true;
    Serial.print("Initial wall distance set to: ");
    Serial.println(initialWallDistance);
  }

  int wallAngle = (leftFront - leftRear) + ANGLE_OFFSET;
  
  if (center <= FRONT_WALL_DISTANCE) {
    moveStop();
    delay(100);
    turnRight();
    delay(500);
    return;
  }

  if (abs(wallAngle) > WALL_FOLLOW_TOLERANCE) {
    if (wallAngle > 0) {
      turnSlightRight();
    } else {
      turnSlightLeft();
    }
  } else if (leftFront > initialWallDistance + WALL_FOLLOW_TOLERANCE) {
    moveSlightLeft();
  } else if (leftFront < initialWallDistance - WALL_FOLLOW_TOLERANCE) {
    moveSlightRight();
  } else {
    moveForward();
  }

  Serial.println("\n=== Navigation Data ===");
  Serial.print("LF: ");
  Serial.print(leftFront);
  Serial.print(" LR: ");
  Serial.print(leftRear);
  Serial.print(" C: ");
  Serial.print(center);
  Serial.print(" Angle: ");
  Serial.print(wallAngle);
  Serial.print(" Target: ");
  Serial.println(initialWallDistance);
  
  delay(40);
}

void moveStop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
} 

void moveForward() {
  if(!goesForward) {
    goesForward=true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);
    motor3.run(FORWARD); 
    motor4.run(FORWARD);     
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward=false;
  motor1.run(BACKWARD);      
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}  

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);     
  delay(TURN_DELAY);
  moveForward();
} 

void turnLeft() {
  motor1.run(BACKWARD);     
  motor2.run(BACKWARD);  
  motor3.run(FORWARD);
  motor4.run(FORWARD);   
  delay(TURN_DELAY);
  moveForward();
}

void turnSlightRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);     
  delay(100);
  moveForward();
}

void turnSlightLeft() {
  motor1.run(BACKWARD);     
  motor2.run(BACKWARD);  
  motor3.run(FORWARD);
  motor4.run(FORWARD);   
  delay(100);
  moveForward();
}

void moveSlightLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED * 0.7);
  motor4.setSpeed(MAX_SPEED * 0.7);
}

void moveSlightRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  
  motor1.setSpeed(MAX_SPEED * 0.7);
  motor2.setSpeed(MAX_SPEED * 0.7);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);
}