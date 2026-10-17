#include <Wire.h>
#include <VL53L0X.h>

// Create VL53L0X objects
VL53L0X sensorRightFront;
VL53L0X sensorRightRear;
VL53L0X sensorCenter;

// XSHUT pins using analog pins as digital
const uint8_t SensorRightFrontXshut = A0;  // Left front sensor
const uint8_t SensorRightearXshut = A1;   // Left rear sensor
const uint8_t SensorCenterXshut = A2;     // Center sensor

// I2C addresses for sensors
const uint8_t SensorRightFrontAddress = 0x30;
const uint8_t SensorRightearAddress = 0x31;
const uint8_t SensorCenterAddress = 0x32;

// Angle offset constant
const int ANGLE_OFFSET = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("VL53L0X Sensor Test with 3-degree offset");
  Serial.println("----------------------------------------");
  
  // Set all XSHUT pins as outputs
  pinMode(SensorRightFrontXshut, OUTPUT);
  pinMode(SensorRightRearXshut, OUTPUT);
  pinMode(SensorCenterXshut, OUTPUT);
  
  // Turn off all sensors
  digitalWrite(SensorRightFrontXshut, LOW);
  digitalWrite(SensorRightRearXshut, LOW);
  digitalWrite(SensorCenterXshut, LOW);
  delay(10);
  
  // Initialize Left Front sensor
  digitalWrite(SensorRightFrontXshut, HIGH);
  delay(10);
  if (!sensorRightFront.init()) {
    Serial.println("Failed to detect Left Front sensor!");
  } else {
    sensorRightFront.setAddress(SensorLeftFrontAddress);
    Serial.println("Left Front sensor initialized at 0x30");
  }
  
  // Initialize Left Rear sensor
  digitalWrite(SensorRightRearXshut, HIGH);
  delay(10);
  if (!sensorRightRear.init()) {
    Serial.println("Failed to detect Left Rear sensor!");
  } else {
    sensorRightRear.setAddress(SensorLeftRearAddress);
    Serial.println("Left Rear sensor initialized at 0x31");
  }
  
  // Initialize Center sensor
  digitalWrite(SensorCenterXshut, HIGH);
  delay(10);
  if (!sensorCenter.init()) {
    Serial.println("Failed to detect Center sensor!");
  } else {
    sensorCenter.setAddress(SensorCenterAddress);
    Serial.println("Center sensor initialized at 0x32");
  }
  
  // Start continuous readings
  sensorRightFront.startContinuous();
  sensorRightRear.startContinuous();
  sensorCenter.startContinuous();
}

void loop() {
  // Read raw values
  int rawLeftFront = sensorLeftFront.readRangeContinuousMillimeters();
  int rawLeftRear = sensorLeftRear.readRangeContinuousMillimeters();
  int rawCenter = sensorCenter.readRangeContinuousMillimeters();
  
  // Convert to cm and adjust for 3cm minimum
  int leftFront = max(0, (rawLeftFront / 10) - 3);
  int leftRear = max(0, (rawLeftRear / 10) - 3);
  int center = max(0, (rawCenter / 10) - 3);
  
  // Calculate angle between left sensors with offset
  int rawAngle = leftFront - leftRear;
  int adjustedAngle = rawAngle + ANGLE_OFFSET;
  
  // Print formatted output
  Serial.println("\n=== Sensor Readings ===");
  
  // Left Front sensor
  Serial.println("Left Front Sensor:");
  Serial.print("  Raw: ");
  Serial.print(rawLeftFront);
  Serial.print(" mm  |  Adjusted: ");
  Serial.print(leftFront);
  Serial.println(" cm");
  
  // Left Rear sensor
  Serial.println("Left Rear Sensor:");
  Serial.print("  Raw: ");
  Serial.print(rawLeftRear);
  Serial.print(" mm  |  Adjusted: ");
  Serial.print(leftRear);
  Serial.println(" cm");
  
  // Center sensor
  Serial.println("Center Sensor:");
  Serial.print("  Raw: ");
  Serial.print(rawCenter);
  Serial.print(" mm  |  Adjusted: ");
  Serial.print(center);
  Serial.println(" cm");
  
  // Angle calculations
  Serial.println("\nAngle Calculations:");
  Serial.print("  Raw Angle: ");
  Serial.print(rawAngle);
  Serial.println(" degrees");
  Serial.print("  With Offset: ");
  Serial.print(adjustedAngle);
  Serial.println(" degrees");
  
  // Check for timeouts
  if (sensorLeftFront.timeoutOccurred()) Serial.println("\nWARNING: Left Front sensor timeout!");
  if (sensorLeftRear.timeoutOccurred()) Serial.println("WARNING: Left Rear sensor timeout!");
  if (sensorCenter.timeoutOccurred()) Serial.println("WARNING: Center sensor timeout!");
  
  Serial.println("\n--------------------");
  delay(5000);
}