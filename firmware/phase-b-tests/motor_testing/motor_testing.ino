// Motor test sketch for Arduino Due + 2x TB6612

// Left motors
const uint8_t LF_IN1 = 22;
const uint8_t LF_IN2 = 23;

const uint8_t LF_PWM = 6;

const uint8_t LR_IN1 = 24;
const uint8_t LR_IN2 = 25;
const uint8_t LR_PWM = 7;

// Right motors
const uint8_t RF_IN1 = 26;
const uint8_t RF_IN2 = 27;
const uint8_t RF_PWM = 8;

const uint8_t RR_IN1 = 28;
const uint8_t RR_IN2 = 29;
const uint8_t RR_PWM = 9;

// Direction correction flags
const bool INVERT_LF = true;   // Left Front inverted
const bool INVERT_LR = true;  // Left Rear normal
const bool INVERT_RF = false;  // Right Front normal
const bool INVERT_RR = false;   // Right Rear inverted

// Motor control function
void setMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t speed, bool invert) {
  if (invert) {
    speed = -speed;
  }
  
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
  
  // Debug output
  Serial.print("Motor pins(");
  Serial.print(in1);
  Serial.print(",");
  Serial.print(in2);
  Serial.print(",");
  Serial.print(pwmPin);
  Serial.print(") Speed:");
  Serial.println(speed);
}

void stopAll() {
  setMotor(LF_IN1, LF_IN2, LF_PWM, 0, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, 0, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, 0, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 0, INVERT_RR);
}

void testMotor(const char* name, uint8_t in1, uint8_t in2, uint8_t pwm, int speed, bool invert) {
  Serial.print("Testing "); 
  Serial.println(name);
  
  setMotor(in1, in2, pwm, speed, invert);
  delay(1500);
  setMotor(in1, in2, pwm, -speed, invert);
  delay(1500);
  setMotor(in1, in2, pwm, 0, invert);
  delay(500);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize all pins
  pinMode(LF_IN1, OUTPUT); digitalWrite(LF_IN1, LOW);
  pinMode(LF_IN2, OUTPUT); digitalWrite(LF_IN2, LOW);
  pinMode(LF_PWM, OUTPUT); analogWrite(LF_PWM, 0);
  
  pinMode(LR_IN1, OUTPUT); digitalWrite(LR_IN1, LOW);
  pinMode(LR_IN2, OUTPUT); digitalWrite(LR_IN2, LOW);
  pinMode(LR_PWM, OUTPUT); analogWrite(LR_PWM, 0);
  
  pinMode(RF_IN1, OUTPUT); digitalWrite(RF_IN1, LOW);
  pinMode(RF_IN2, OUTPUT); digitalWrite(RF_IN2, LOW);
  pinMode(RF_PWM, OUTPUT); analogWrite(RF_PWM, 0);
  
  pinMode(RR_IN1, OUTPUT); digitalWrite(RR_IN1, LOW);
  pinMode(RR_IN2, OUTPUT); digitalWrite(RR_IN2, LOW);
  pinMode(RR_PWM, OUTPUT); analogWrite(RR_PWM, 0);

  stopAll();
  Serial.println("Motor test initialized");
  delay(1000);
}

void loop() {
  const int speed = 160;

  // All motors forward
  Serial.println("All forward");
  setMotor(LF_IN1, LF_IN2, LF_PWM, speed, INVERT_LF);
  setMotor(LR_IN1, LR_IN2, LR_PWM, speed, INVERT_LR);
  setMotor(RF_IN1, RF_IN2, RF_PWM, speed, INVERT_RF);
  setMotor(RR_IN1, RR_IN2, RR_PWM, speed, INVERT_RR);
  delay(2000);
  stopAll();


}