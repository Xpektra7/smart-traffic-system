// ultrasonic_avg_speed.cpp
#include <Arduino.h>

// pins
constexpr int pingPin = 5;
constexpr int echoPin = 18;

// timing
constexpr unsigned long WINDOW_DURATION_MS = 30000UL; // 30s
constexpr unsigned long SAMPLE_INTERVAL_MS = 50UL;    // 20 Hz

// detection params
constexpr float DROP_THRESHOLD_M = 0.5f;   // start of vehicle dip
constexpr float RISE_THRESHOLD_M = 0.2f;   // return to baseline
constexpr float MAX_DISTANCE_M = 5.0f;     // sensor max range fallback (meters)

// runtime state
unsigned long windowStart_ms = 0;
unsigned long lastSample_ms = 0;
float baselineDist = 0.0f;
bool hasBaseline = false;

bool carPresent = false;
unsigned long carStart_ms = 0;
int vehicleCount = 0;
float totalOccupiedSeconds = 0.0f;

float prevDist = -1.0f;

// speed accumulation
float slopeSum = 0.0f;
int slopeSamples = 0;

// utils
float microsecondsToMeters(unsigned long us) {
    return (us / 29.0f / 2.0f) / 100.0f; // convert µs → meters
}

float readDistanceMeters() {
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);

    unsigned long dur = pulseIn(echoPin, HIGH, 30000UL); // timeout ~5m
    if (dur == 0) return MAX_DISTANCE_M;
    return microsecondsToMeters(dur);
}

String classifySpeed(float slope) {
    if (slope < 0.5f) return "slow";
    if (slope < 1.5f) return "faster";
    return "fastest";
}

void printFinalResults() {
    Serial.println("---- 30s Results ----");
    Serial.print("Vehicle Count: ");
    Serial.println(vehicleCount);

    float occupancyRatio = totalOccupiedSeconds / (WINDOW_DURATION_MS / 1000.0f);
    Serial.print("Occupancy Ratio: ");
    Serial.println(occupancyRatio, 2);

    Serial.print("Baseline Dist: ");
    Serial.println(baselineDist, 2);

    if (slopeSamples > 0) {
        float avgSlope = slopeSum / slopeSamples;
        Serial.print("Average Rel Speed: ");
        Serial.print(avgSlope, 3);
        Serial.print(" m/s | Category: ");
        Serial.println(classifySpeed(avgSlope));
    } else {
        Serial.println("Average Rel Speed: N/A");
    }

    Serial.println("----------------------");
}

void setup() {
    Serial.begin(9600);
    pinMode(pingPin, OUTPUT);
    pinMode(echoPin, INPUT);
    windowStart_ms = millis();
    lastSample_ms = 0;
}

void loop() {
    unsigned long now = millis();

    if (now - windowStart_ms >= WINDOW_DURATION_MS) {
        printFinalResults();
        while (true) { /* halt */ }
    }

    if (now - lastSample_ms < SAMPLE_INTERVAL_MS) return;
    lastSample_ms = now;

    float dist = readDistanceMeters();
    Serial.print("Distance: ");
    Serial.print(dist, 2);
    Serial.println(" m");

    if (!carPresent && dist < MAX_DISTANCE_M && dist > baselineDist + 0.01f) {
        baselineDist = dist;
        hasBaseline = true;
    }

    float slope = 0.0f;
    if (prevDist >= 0.0f) {
        float dt_s = SAMPLE_INTERVAL_MS / 1000.0f;
        slope = fabs(dist - prevDist) / dt_s;
    }

    if (slope > 0.0f) {
        slopeSum += slope;
        slopeSamples++;
    }

    if (!carPresent) {
        if (hasBaseline && dist < (baselineDist - DROP_THRESHOLD_M)) {
            carPresent = true;
            carStart_ms = now;
            vehicleCount++;
            Serial.println("Vehicle detected!");
        }
    } else {
        if (hasBaseline && dist >= (baselineDist - RISE_THRESHOLD_M)) {
            unsigned long carEnd_ms = now;
            totalOccupiedSeconds += (carEnd_ms - carStart_ms) / 1000.0f;
            Serial.println("Vehicle left.");
            carPresent = false;
        }
    }

    prevDist = dist;
}

int main() {
    init();       // Arduino core init
    setup();
    while (true) {
        loop();
    }
    return 0;
}
