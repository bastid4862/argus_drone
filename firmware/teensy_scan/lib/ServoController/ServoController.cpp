#include "ServoController.hpp"

// The constructor saves the pin number you chose
ServoController::ServoController(uint8_t pin) {
    servo_pin_ = pin;
}

void ServoController::begin() {
    my_servo_.attach(servo_pin_);
}

void ServoController::setAngle(float angle) {
    my_servo_.write(angle);
}