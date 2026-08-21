#include <Arduino.h>
#include "Encoder.hpp"
#include "ServoController.hpp"

// Use Pin 10 as our CS (Chip Select) wire.
Encoder myEncoder(10);
// Servo signal pin
ServoController myServo(23);

void setup() {
    // Start the USB serial connection so we can read text on our computer screen
    Serial.begin(115200);

    // Run the hardware setup we wrote in Encoder.cpp
    myEncoder.begin();
    myServo.begin();
}

void loop() {
    // Sweep UP from 0 to 180 degrees
    for (int pos = 0; pos <= 180; pos++) {
        myServo.setAngle(pos);
        Serial.println(myEncoder.readAngle());
        delay(15);
    }

    // Sweep DOWN from 180 to 0 degrees
    for (int pos = 180; pos >= 0; pos--) {
        myServo.setAngle(pos);
        Serial.println(myEncoder.readAngle());
        delay(15);
    }
}
