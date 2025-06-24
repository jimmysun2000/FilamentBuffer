#include <Arduino.h>
#include "buffer.hpp"

void setup() {
    Serial.begin(115200);
    Serial.dtr(false);
    buffer.begin(); 
}

void loop() {
    buffer.update(); 
}
