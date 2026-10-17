#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.setWire(&Wire);

    Serial.println("Initializing MPU9250...");
    
    // Initialize sensors - the begin functions don't return values
    mpu.beginAccel();
    mpu.beginGyro();
    mpu.beginMag();

    // The library doesn't support direct range setting
    // Default ranges are: Accel ±2G, Gyro ±250deg/sec

    Serial.println("MPU9250 initialized!");
}

void loop() {
    if (mpu.accelUpdate() == 0) {  // Returns 0 on success
        Serial.print("Accel: ");
        Serial.print(mpu.accelX(), 3);
        Serial.print(" ");
        Serial.print(mpu.accelY(), 3);
        Serial.print(" ");
        Serial.print(mpu.accelZ(), 3);
        Serial.print(" G ");
    }

    if (mpu.gyroUpdate() == 0) {   // Returns 0 on success
        Serial.print("Gyro: ");
        Serial.print(mpu.gyroX(), 3);
        Serial.print(" ");
        Serial.print(mpu.gyroY(), 3);
        Serial.print(" ");
        Serial.print(mpu.gyroZ(), 3);
        Serial.println(" deg/s");
    }

    delay(100);
}