#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <SPI.h>

class Encoder {
public:
    // Takes chosen CS pin as a setup parameter.
    Encoder(uint8_t cs_pin);

    // We will call this once to set up the hardware pins.
    void begin();

    // We will call this whenever we want the current angle.
    float readAngle();

private:
    // Stores the pin number.
    uint8_t cs_pin_;
    // Add the correct parity bit
    uint16_t addParity(uint16_t value);
};

#endif