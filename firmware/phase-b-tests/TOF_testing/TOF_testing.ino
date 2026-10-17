#include <Wire.h>
#include <VL53L0X.h>

// Create sensor objects
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// Define XSHUT pins
#define XSHUT_1 2
#define XSHUT_2 3
#define XSHUT_3 4

// I2C addresses for sensors
#define ADDRESS_1 0x30
#define ADDRESS_2 0x31
#define ADDRESS_3 0x32

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Configure XSHUT pins as outputs
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  
  // Turn off all sensors
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);
  
  // Initialize sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setAddress(ADDRESS_1);
  
  // Initialize sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setAddress(ADDRESS_2);
  
  // Initialize sensor 3
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  sensor3.init();
  sensor3.setAddress(ADDRESS_3);
  
  // Start continuous measurements
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
}

void loop() {
  // Read measurements from all sensors
  int distance1 = sensor1.readRangeContinuousMillimeters();
  int distance2 = sensor2.readRangeContinuousMillimeters();
  int distance3 = sensor3.readRangeContinuousMillimeters();
  
  // Print results
  Serial.print("Sensor 1: ");
  Serial.print(distance1);
  Serial.print("mm, Sensor 2: ");
  Serial.print(distance2);
  Serial.print("mm, Sensor 3: ");
  Serial.print(distance3);
  Serial.println("mm");
  
  // Check for timeout errors
  if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred() || sensor3.timeoutOccurred()) {
    Serial.println("Timeout error occurred!");
  }
  
  delay(100);  // Small delay between readings
}