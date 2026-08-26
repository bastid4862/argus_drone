#pragma once

#include "Encoder.hpp"
#include "ServoController.hpp"
#include <cstdint>

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

        float homeSpeed_ = 10.0f;
        float positionSpeed_ = 20.0f;
        float sweepSpeed_ = 20.0f;

        float minAngle_ = 0.0f;
        float maxAngle_ = 180.0f;
        int8_t sweepDirection_ = 1;

        unsigned long lastUpdateTime_ = 0;

        float homeAngle_ = 0.0f;

        float mechanicalMin_ = 0.0f;
        float mechanicalMax_ = 180.0f;

    public:
        ScanController(ServoController& servo, Encoder& encoder);

        bool hasFault() const { return hasFault_; }
        ScanMode getMode() const {return currentMode_;}
        void setMode(ScanMode mode);
        void update();
        void setTargetAngle(float angle);
        void setSweepLimits(float min_angle, float max_angle);
        void setSweepSpeed(float speed);
        void clearFault();
        void setSweepDirection(int8_t direction);
        bool configureSweep(float min_angle, float max_angle, float speed, int8_t direction);
        void setFault();

        float getTargetAngle() const {
            return targetAngle_;
        }

        int8_t getSweepDirection() const {
            return sweepDirection_;
        }

};

