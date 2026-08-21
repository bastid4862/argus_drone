#include <Arduino.h>
#include "Encoder.hpp"

// Use Pin 10 as our CS (Chip Select) wire.
Encoder myEncoder(10);

void setup() {
    // Start the USB serial connection so we can read text on our computer screen
    Serial.begin(115200);

    // Run the hardware setup we wrote in Encoder.cpp
    myEncoder.begin();
}

void loop() {
    // Reads and prinst angle
    Serial.println(myEncoder.readAngle());
    // Add a small delay
    delay(50);
}
