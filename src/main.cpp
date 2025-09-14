#include <Arduino.h>

struct UltrasonicResult {
    int vehicleCount;
    float occupancyRatio;
    float avgSpeed;
};

UltrasonicResult runUltrasonicWindow(int seconds, int trigPin, int echoPin) {
    const int sampleInterval = 50; // ms -> 20Hz
    unsigned long startTime = millis();
    unsigned long elapsed = 0;

    float baselineDist = 0;
    int baselineSamples = 0;

    int vehicleCount = 0;
    float occupiedTime = 0;

    bool carPresent = false;
    unsigned long carStartTime = 0;

    float totalSpeed = 0;
    int speedSamples = 0;

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    while (elapsed < (unsigned long)(seconds * 1000)) {
        // trigger pulse
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // read echo
        long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
        float distance = duration / 58.0; // cm

        // update baseline if valid
        if (distance > 0 && distance < 500) {
            if (baselineDist < distance) {
                baselineDist = distance;
            }
        }

        // detect car presence
        if (!carPresent && distance > 0 && distance < (baselineDist - 20)) {
            carPresent = true;
            carStartTime = millis();
            vehicleCount++;
        }

        // detect car leaving
        if (carPresent && distance > (baselineDist - 10)) {
            carPresent = false;
            unsigned long carEndTime = millis();
            occupiedTime += (carEndTime - carStartTime) / 1000.0;
        }

        // crude speed estimate: slope of distance change
        static float lastDist = 0;
        static unsigned long lastTime = 0;
        if (lastDist > 0 && distance > 0) {
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0;
            if (dt > 0) {
                float slope = fabs(distance - lastDist) / dt;
                totalSpeed += slope;
                speedSamples++;
            }
            lastTime = now;
            lastDist = distance;
        } else {
            lastDist = distance;
            lastTime = millis();
        }

        delay(sampleInterval);
        elapsed = millis() - startTime;
    }

    // final occupancy ratio
    float occupancyRatio = occupiedTime / (float)seconds;
    float avgSpeed = (speedSamples > 0) ? (totalSpeed / speedSamples) : 0;

    UltrasonicResult res = { vehicleCount, occupancyRatio, avgSpeed };
    return res;
}

// Example usage in Arduino setup()
void setup() {
    Serial.begin(115200);

    UltrasonicResult res = runUltrasonicWindow(30, 5, 18);

    Serial.println("---- Results ----");
    Serial.print("Vehicle Count: "); Serial.println(res.vehicleCount);
    Serial.print("Occupancy Ratio: "); Serial.println(res.occupancyRatio, 2);
    Serial.print("Avg Speed: "); Serial.println(res.avgSpeed, 2);
    Serial.println("-----------------");

    while (true) {} // stop after one run
}

void loop() {
    // not used
}
