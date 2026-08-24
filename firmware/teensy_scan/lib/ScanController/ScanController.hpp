#pragma once

#include "Encoder.hpp"
#include "ServoController.hpp"

enum class ScanMode {
    STOPPED,
    HOME,
    HOLD,
    POSITION,
    SWEEP,
};

class ScanController {
    private:
        ServoController& servo_;
        Encoder& encoder_;
        ScanMode currentMode_ = ScanMode::HOME;

        bool hasFault_ = false;
        bool initialized_ = false;

        float targetAngle_ = 180.0f;
        float currentAngle_ = 0.0f;
        float stepSize_ = 1.0f;

        float minAngle_ = 0.0f;
        float maxAngle_ = 180.0f;

        float getTargetAngle() const { return targetAngle_; }




    public:
        ScanController(ServoController& servo, Encoder& encoder);

        bool hasFault() const { return hasFault_; }
        ScanMode getMode() const {return currentMode_;}
        void setMode(ScanMode mode);
        void update();
        void setTargetAngle(float angle);
};

