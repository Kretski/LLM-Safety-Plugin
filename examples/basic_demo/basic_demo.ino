#include <Arduino.h>
#include "MicroSafeController.h"

MicroSafeController safe;

// симулиран вход (можеш да го замениш със сензор)
float fake_sensor() {
    // лек шум + случайни скокове
    float base = sin(millis() * 0.002f);

    if (random(0, 100) > 95) {
        base += random(-200, 200) / 100.0f; // spike
    }

    return base;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ai_input,safe_output,safety_score"); // CSV header
}

void loop() {
    // 1. AI action (симулация)
    float ai = random(-200, 200) / 100.0f;

    // 2. Sensor value
    float sensor = fake_sensor();

    // 3. Safe control
    float safe_cmd = safe.apply(ai, sensor);

    // 4. CSV лог (за matplotlib)
    Serial.print(ai); Serial.print(",");
    Serial.print(safe_cmd); Serial.print(",");
    Serial.println(safe.get_safety_score());

    delay(50); // ~20Hz
}