// Maze solving and obstacle avoiding robot
// Arduino Mega + 2x TB6612 + 3x Ultrasonic + MPU9250

#include <MPU9250_asukiaaa.h>

// ===== MOTOR PINS (Arduino Mega) =====
const uint8_t LF_IN1 = 22, LF_IN2 = 23, LF_PWM = 6;
const uint8_t LR_IN1 = 24, LR_IN2 = 25, LR_PWM = 7;
const uint8_t RF_IN1 = 26, RF_IN2 = 27, RF_PWM = 8;
const uint8_t RR_IN1 = 28, RR_IN2 = 29, RR_PWM = 9;

const bool INVERT_LF = true, INVERT_LR = true;
const bool INVERT_RF = false, INVERT_RR = false;

// ===== TB6612 STBY PIN =====
const uint8_t TB_STBY = 5;

// ===== ULTRASONIC PINS (Arduino Mega) =====
const uint8_t TRIG_FRONT = 2, ECHO_FRONT = 11;
const uint8_t TRIG_LEFT = 3, ECHO_LEFT = 12;
const uint8_t TRIG_RIGHT = 4, ECHO_RIGHT = 13;

// ===== MPU9250 =====
MPU9250_asukiaaa mpu;

// ===== SENSOR DATA STRUCTURE =====
struct SensorData {
  float front, left, right;
  float accelX, accelY, accelZ;
};

// ===== ROBOT STATE =====
enum RobotState {
  EXPLORING,
  OBSTACLE_DETECTED,
  TURNING,
  WALL_FOLLOWING
};

RobotState currentState = EXPLORING;

// ===== PARAMETERS =====
const float OBSTACLE_DISTANCE = 15.0;  // cm - emergency stop distance for front
const float SIDE_OBSTACLE_DISTANCE = 12.0;  // cm - side obstacle threshold
const float SAFE_DISTANCE = 20.0;  // cm - safe distance for turning
const uint8_t SPEED = 250;              // motor speed (0-255)
const uint8_t TURN_SPEED = 150;         // turning speed
const uint8_t CORRECTION_SPEED = 30;   // correction speed for slight adjustments

// ===== TIMING =====
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 100; // ms

// ===== FUNCTION: Set motor speed =====
void setMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t speed, bool invert) {
  if (invert) speed = -speed;
  
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, constrain(speed, 0, 255));
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, constrain(-speed, 0, 255));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// ===== FUNCTION: Stop all motors =====
void stopAll() {
  setMotor(LF_IN1, LF_IN2, LF_PWM, 0, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, 0, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, 0, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 0, INVERT_RR);
}

// ===== FUNCTION: Move forward =====
void moveForward(uint8_t speed) {
  setMotor(LF_IN1, LF_IN2, LF_PWM, speed, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, speed, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, speed, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, speed, INVERT_RR);
}

// ===== FUNCTION: Turn left =====
void turnLeft(uint8_t speed) {
  setMotor(LF_IN1, LF_IN2, LF_PWM, -speed, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, -speed, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, speed, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, speed, INVERT_RR);
}

// ===== FUNCTION: Turn right =====
void turnRight(uint8_t speed) {
  setMotor(LF_IN1, LF_IN2, LF_PWM, speed, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, speed, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, -speed, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, -speed, INVERT_RR);
}

// ===== FUNCTION: Slight left correction =====
void slightLeft(uint8_t speed) {
  setMotor(LF_IN1, LF_IN2, LF_PWM, speed - CORRECTION_SPEED, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, speed - CORRECTION_SPEED, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, speed + CORRECTION_SPEED, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, speed + CORRECTION_SPEED, INVERT_RR);
}

// ===== FUNCTION: Slight right correction =====
void slightRight(uint8_t speed) {
  setMotor(LF_IN1, LF_IN2, LF_PWM, speed + CORRECTION_SPEED, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, speed + CORRECTION_SPEED, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, speed - CORRECTION_SPEED, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, speed - CORRECTION_SPEED, INVERT_RR);
}

// ===== FUNCTION: Read ultrasonic distance =====
float readDistance(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  
  return duration / 58.0;
}

// ===== FUNCTION: Read all sensors =====
SensorData readAllSensors() {
  SensorData data;
  
  // Read ultrasonic sensors
  data.front = readDistance(TRIG_FRONT, ECHO_FRONT);
  delay(10);
  data.left = readDistance(TRIG_LEFT, ECHO_LEFT);
  delay(10);
  data.right = readDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // Read MPU9250 (optional for heading/tilt correction)
  if (mpu.accelUpdate() == 0) {
    data.accelX = mpu.accelX();
    data.accelY = mpu.accelY();
    data.accelZ = mpu.accelZ();
  }
  
  return data;
}

// ===== FUNCTION: Log sensor data =====
void logSensors(const SensorData& data) {
  Serial.print("Front: ");
  Serial.print(data.front, 1);
  Serial.print(" cm | Left: ");
  Serial.print(data.left, 1);
  Serial.print(" cm | Right: ");
  Serial.print(data.right, 1);
  Serial.println(" cm");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize TB6612 STBY pin (enable drivers)
  pinMode(TB_STBY, OUTPUT);
  digitalWrite(TB_STBY, HIGH);
  delay(100);
  
  // Initialize motor pins
  pinMode(LF_IN1, OUTPUT); pinMode(LF_IN2, OUTPUT); pinMode(LF_PWM, OUTPUT);
  pinMode(LR_IN1, OUTPUT); pinMode(LR_IN2, OUTPUT); pinMode(LR_PWM, OUTPUT);
  pinMode(RF_IN1, OUTPUT); pinMode(RF_IN2, OUTPUT); pinMode(RF_PWM, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_PWM, OUTPUT);
  
  // Initialize ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize MPU9250
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  
  stopAll();
  Serial.println("=== MAZE SOLVER INITIALIZED ===");
  Serial.println("TB6612 STBY enabled, motors ready");
  delay(2000);
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long now = millis();
  
  if (now - lastSensorRead < SENSOR_INTERVAL) {
    return; // Rate limit sensor reads
  }
  lastSensorRead = now;
  
  SensorData sensors = readAllSensors();
  logSensors(sensors);
  
  // ===== STATE MACHINE: Multi-sensor directional navigation =====
  
  // Priority 1: Check for immediate front obstacle
  if (sensors.front > 0 && sensors.front < OBSTACLE_DISTANCE) {
    Serial.println("FRONT OBSTACLE DETECTED!");
    stopAll();
    delay(200);
    
    // Evaluate left vs right for escape
    if (sensors.left > SAFE_DISTANCE && sensors.right > SAFE_DISTANCE) {
      // Both sides clear - prefer right turn first
      Serial.println("Both sides clear, turning RIGHT");
      turnRight(TURN_SPEED);
    } else if (sensors.left > SAFE_DISTANCE) {
      Serial.println("Left side clear, turning LEFT");
      turnLeft(TURN_SPEED);
    } else if (sensors.right > SAFE_DISTANCE) {
      Serial.println("Right side clear, turning RIGHT");
      turnRight(TURN_SPEED);
    } else {
      // Both blocked - reverse
      Serial.println("Both sides blocked, reversing...");
      setMotor(LF_IN1, LF_IN2, LF_PWM, -SPEED, INVERT_LF);
      setMotor(LR_IN1, LR_IN2, LR_PWM, -SPEED, INVERT_LR);
      setMotor(RF_IN1, RF_IN2, RF_PWM, -SPEED, INVERT_RF);
      setMotor(RR_IN1, RR_IN2, RR_PWM, -SPEED, INVERT_RR);
    }
    delay(600);
    return;
  }
  
  // Priority 2: Side obstacle detection - turn away from obstacles
  if (sensors.right > 0 && sensors.right < SIDE_OBSTACLE_DISTANCE) {
    // Obstacle on right - turn LEFT
    Serial.println("Obstacle on RIGHT side, turning LEFT");
    turnLeft(TURN_SPEED);
    delay(400);
    return;
  }
  
  if (sensors.left > 0 && sensors.left < SIDE_OBSTACLE_DISTANCE) {
    // Obstacle on left - turn RIGHT
    Serial.println("Obstacle on LEFT side, turning RIGHT");
    turnRight(TURN_SPEED);
    delay(400);
    return;
  }
  
  // Priority 3: Fine corrections based on side distances (wall following)
  if (sensors.right > 0 && sensors.left > 0) {
    float rightDist = sensors.right;
    float leftDist = sensors.left;
    float distDiff = rightDist - leftDist;
    
    if (distDiff > 3.0) {
      // Right side too close, correct left
      Serial.println("Correcting LEFT (right too close)");
      slightLeft(SPEED);
    } else if (distDiff < -3.0) {
      // Left side too close, correct right
      Serial.println("Correcting RIGHT (left too close)");
      slightRight(SPEED);
    } else {
      // Good balance, move straight
      Serial.println("Straight ahead (balanced)");
      moveForward(SPEED);
    }
  } else {
    // Default: move forward if sensor data incomplete
    moveForward(SPEED);
  }
}