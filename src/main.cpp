#include <Arduino.h>
#include "buffer.h"

void setup() {
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