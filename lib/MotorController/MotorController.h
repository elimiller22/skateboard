#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include <mbed.h>

using namespace mbed;

const short defaultRampSteps = 1;
const short defaultRampDelay_ms = 5;

class MotorController {
   public:
    MotorController(unsigned short forwardPin);  // todo - add to programming if reverse not included
    MotorController(unsigned short forwardPin, unsigned short reversePin);
    MotorController(unsigned short forwardPin, unsigned short reversePin, unsigned short enablePin);  // todo - Add enablePin functionality

    void setSpeed(short speedPercentage);
    float getSpeed();
    // void setRampRate(float rampRatePerSec);
    void setFrequency(unsigned short pwmFrequency);
    void enableRampRate();
    void disableRampRate();

   private:
    PwmOut *forwardPin;
    PwmOut *reversePin;
    DigitalOut *enablePin;
    bool rampRateEnabled;
    short rampSteps;
    short rampDelay;
    unsigned long timeAtLastSet;
    short currentSpeed;
};

#endif