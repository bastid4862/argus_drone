#include "Encoder.hpp"
#include <Arduino.h>
#include <SPI.h>

// The constructor saves the pin number you chose
Encoder::Encoder(uint8_t cs_pin) {
    cs_pin_ = cs_pin;
}

void Encoder::begin() {
    // Tell the Teensy this pin will send voltage OUT
    pinMode(cs_pin_, OUTPUT);

    // Set the voltage HIGH (sensor asleep)
    digitalWrite(cs_pin_, HIGH);

    // Turn on the Teensy's SPI hardware
    SPI.begin();
}

uint16_t Encoder::addParity(uint16_t value) {
    // Count how many 1 bits are in bits 0-14
    uint16_t temp = value;
    bool parity = false;

    while (temp) {
        parity = !parity;
        temp &= (temp - 1);
    }

    // If there are an odd number of 1s,
    // set bit 15 so the total becomes even
    if (parity) {
        value |= 0x8000;
    }

    return value;
}

float Encoder::readAngle() {
    // ANGLECOM register address
    const uint16_t ANGLE_REGISTER = 0x3FFF;

    // Bit 14 = 1 means READ
    uint16_t command = ANGLE_REGISTER | 0x4000;

    // Add the parity bit
    command = addParity(command);

    // SPI settings for AS5047P
    SPI.beginTransaction(
        SPISettings(1000000, MSBFIRST, SPI_MODE1)
    );

    // ---- First SPI frame ----
    // Ask the encoder for its angle
    digitalWrite(cs_pin_, LOW);

    SPI.transfer16(command);

    digitalWrite(cs_pin_, HIGH);

    // ---- Second SPI frame ----
    // The encoder gives us the angle here
    digitalWrite(cs_pin_, LOW);

    uint16_t response = SPI.transfer16(0x0000);

    digitalWrite(cs_pin_, HIGH);

    SPI.endTransaction();

    // Keep only the 14 angle bits
    uint16_t raw_angle = response & 0x3FFF;

    // Convert 0-16383 into 0-360 degrees
    float angle_deg =
        (raw_angle / 16384.0f) * 360.0f;

    return angle_deg;
}