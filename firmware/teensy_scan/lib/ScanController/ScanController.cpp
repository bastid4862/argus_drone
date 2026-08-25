#include "ScanController.hpp"
#include <Arduino.h>
#include <cmath>

ScanController::ScanController(ServoController& servo, Encoder& encoder) : servo_(servo), encoder_(encoder) {

}

void ScanController::setMode(ScanMode mode) {
    currentMode_ = mode;
}

void ScanController::setTargetAngle(float angle) {

    if (angle < mechanicalMin_) {
        angle = mechanicalMin_;
    }

    if (angle > mechanicalMax_) {
        angle = mechanicalMax_;
    }

    targetAngle_ = angle;
}

void ScanController::setSweepLimits(float min_angle, float max_angle) {

    if (min_angle < mechanicalMin_ || max_angle > mechanicalMax_ || min_angle >= max_angle) {
        return;
    }

    minAngle_ = min_angle;
    maxAngle_ = max_angle;
}

void ScanController::setSweepSpeed(float speed) {
    if (speed <= 0.0f) {
        return;
    }

    sweepSpeed_ = speed;
}

void ScanController::setSweepDirection(int8_t direction) {
    if (direction != 1 && direction != -1) {
        return;
    }

    sweepDirection_ = direction;
}

bool ScanController::configureSweep(float min_angle, float max_angle, float speed, int8_t direction) {
    if (min_angle < mechanicalMin_ || max_angle > mechanicalMax_ || min_angle >= max_angle) {
        return false;
    }

    if (direction != 1 && direction != -1) {
        return false;
    }

    if (speed <= 0.0f) {
        return false;
    }

    minAngle_ = min_angle;
    maxAngle_ = max_angle;
    sweepSpeed_ = speed;
    sweepDirection_ = direction;

    return true;
}

void ScanController::setFault() {
    hasFault_ = true;
}

void ScanController::update() {
    if (currentMode_ == ScanMode::STOPPED) {
        lastUpdateTime_ = millis();  // Keep timing fresh while stopped
        servo_.stop();
        return;
    }

    unsigned long currentTime = millis();    // Saves the time

    if (lastUpdateTime_ == 0) {
        lastUpdateTime_ = currentTime;
        return;
    }

    float deltaTime = (currentTime - lastUpdateTime_)/1000.0f;
    lastUpdateTime_ = currentTime;

    // Measure position & run safety check
    float measuredAngle = encoder_.readAngle();

    if (!initialized_) {
        currentAngle_ = measuredAngle;
        initialized_ = true;
    }

    if (std::abs(currentAngle_ - measuredAngle) > 15.0f) {
        hasFault_ = true;
        currentMode_ = ScanMode::STOPPED;
        servo_.stop();
        return;
    }

    switch (currentMode_) {
        case ScanMode::STOPPED: {
            servo_.stop();
            return;
        }

        case ScanMode::HOME: {
            float step = homeSpeed_ * deltaTime;

            if (currentAngle_ > homeAngle_) {
                currentAngle_ -= step;
            }
            if (currentAngle_ <= homeAngle_) {
                currentAngle_ = homeAngle_;
                currentMode_ = ScanMode::HOLD;
            }

            break;
        }

        case ScanMode::HOLD: {
            // Do nothing; servo holds its last position automatically
            break;
        }

        case ScanMode::POSITION: {
            float step = positionSpeed_ * deltaTime;

            if (currentAngle_ < targetAngle_) {

                currentAngle_ += step;

                if (currentAngle_ > targetAngle_) {
                    currentAngle_ = targetAngle_;
                }
            }
            else if (currentAngle_ > targetAngle_) {

                currentAngle_ -= step;

                if (currentAngle_ < targetAngle_) {
                    currentAngle_ = targetAngle_;
                }
            }

            break;
        }

        case ScanMode::SWEEP: {
            float step = sweepSpeed_ * deltaTime;

            if (sweepDirection_ == 1) {
                currentAngle_ += step;

                if (currentAngle_ >= maxAngle_) {
                    currentAngle_ = maxAngle_;
                    sweepDirection_ = -1;
                }
            }
            else if (sweepDirection_ == -1) {
                currentAngle_ -= step;

                if (currentAngle_ <= minAngle_) {
                    currentAngle_ = minAngle_;
                    sweepDirection_ = 1;
                }
            }

            break;
        }

    }
    servo_.setAngle(currentAngle_);
}

void ScanController::clearFault() {
    hasFault_ = false;
    currentAngle_ = encoder_.readAngle();
    currentMode_ = ScanMode::HOME;
}