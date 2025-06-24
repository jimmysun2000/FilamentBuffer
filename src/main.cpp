#include <Arduino.h>
#include "buffer.h"

void setup() {
    Serial.begin(115200);
    Serial.dtr(false);
    bufferInit();
}

void loop() {
    bufferLoop();
}
