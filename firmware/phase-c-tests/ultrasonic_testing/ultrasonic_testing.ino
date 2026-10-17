// Arduino sketch: 3 ultrasonic sensors test
// Use on Arduino Mega

const uint8_t trigPins[3] = {2, 3, 4};
const uint8_t echoPins[3] = {11, 12, 13};
const char* labels[3] = {"FRONT", "LEFT ", "RIGHT"};
const unsigned long timeout = 30000UL; // microseconds (30ms -> ~5m max)

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 3; ++i) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
        digitalWrite(trigPins[i], LOW);
    }
    delay(200);
    Serial.println("Ultrasonic test start");
}

float readCm(int idx) {
    uint8_t t = trigPins[idx];
    uint8_t e = echoPins[idx];

    // Trigger pulse
    digitalWrite(t, LOW);
    delayMicroseconds(2);
    digitalWrite(t, HIGH);
    delayMicroseconds(10);
    digitalWrite(t, LOW);

    // Measure echo duration (microseconds)
    unsigned long dur = pulseIn(e, HIGH, timeout);
    if (dur == 0) return -1.0; // timeout / no echo

    // Speed of sound ~343 m/s => 29.1 us per cm for round trip
    float distanceCm = dur / 58.0; // dur/2/29.1 ~= dur/58
    return distanceCm;
}

void loop() {
    for (int i = 0; i < 3; ++i) {
        float cm = readCm(i);
        if (cm < 0) {
            Serial.print(labels[i]);
            Serial.println(": out of range");
        } else {
            Serial.print(labels[i]);
            Serial.print(": ");
            Serial.print(cm, 1);
            Serial.println(" cm");
        }
        delay(60); // small gap between sensors to reduce cross-talk
    }
    Serial.println("---");
    delay(500); // overall loop delay
}