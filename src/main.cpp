#include <Arduino.h>
#include "buffer.h"

void setup() {
    // Disable the driver before anything else: it keeps turning the motor by
    // itself, so after a reset this is what actually stops it
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, HIGH);

    delay(1000);
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("boot");
    bufferInit();
}

void loop() {
    bufferLoop();
}