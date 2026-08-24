#include "ScanController.hpp"
#include <cmath>

ScanController::ScanController(ServoController& servo, Encoder& encoder) : servo_(servo), encoder_(encoder) {

}

void ScanController::setMode(ScanMode mode) {
    currentMode_ = mode;
}

void ScanController::setTargetAngle(float angle) {

    if (angle < minAngle_) {
        angle = minAngle_;
    }

    if (angle > maxAngle_) {
        angle = maxAngle_;
    }

    targetAngle_ = angle;
}

void ScanController::update() {
    if (currentMode_ == ScanMode::STOPPED) {
        servo_.stop();
        return;
    }

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
        case ScanMode::STOPPED:
            servo_.stop();
            return;

        case ScanMode::HOME:
            if (currentAngle_ > minAngle_) {
                currentAngle_ -= stepSize_;
                if (currentAngle_ <= minAngle_) {
                    currentAngle_ = minAngle_;
                    currentMode_ = ScanMode::HOLD;
                }
            }
            break;

        case ScanMode::HOLD:
            // Do nothing; servo holds its last position automatically
            break;

        case ScanMode::POSITION:

            if (currentAngle_ < targetAngle_) {

                currentAngle_ += stepSize_;

                if (currentAngle_ > targetAngle_) {
                    currentAngle_ = targetAngle_;
                }
            }
            else if (currentAngle_ > targetAngle_) {

                currentAngle_ -= stepSize_;

                if (currentAngle_ < targetAngle_) {
                    currentAngle_ = targetAngle_;
                }
            }

            break;

        case ScanMode::SWEEP:

            if (currentAngle_ < targetAngle_) {
                currentAngle_ += stepSize_;
                if (currentAngle_ >= maxAngle_) {
                    currentAngle_ = maxAngle_;
                    targetAngle_ = minAngle_;
                }
            }
            else if (currentAngle_ > targetAngle_) {
                currentAngle_ -= stepSize_;
                if (currentAngle_ <= minAngle_) {
                    currentAngle_ = minAngle_;
                    targetAngle_ = maxAngle_;
                }
            }
            break;

    }
    servo_.setAngle(currentAngle_);
}