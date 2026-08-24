#ifndef SERVOCONTROLLER_HPP
#define SERVOCONTROLLER_HPP

#include <Arduino.h>
#include <Servo.h> // Our new built-in library!

class ServoController {
public:
    // Pin number where the servo signal wire is connected
    ServoController(uint8_t pin);

    // Setup function to initialize the hardware
    void begin();

    // Function to command the motor to a specific position
    void setAngle(float angle);

    void stop();

private:
    uint8_t servo_pin_;
    Servo my_servo_;
};

#endif