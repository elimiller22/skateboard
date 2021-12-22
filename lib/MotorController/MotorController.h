#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>

#include "mbed.h"

// using namespace mbed;

class MotorController {
   public:
    MotorController(unsigned short forwardPin);
    // MotorController(unsigned short int forwardPin, unsigned short int reversePin);
    // MotorController(unsigned short int forwardPin, unsigned short int reversePin, unsigned short int enablePin);

    void setSpeed(unsigned short speedPercentage);
    // void setRampRate(float rampRatePerSec);
    void setFrequency(unsigned short pwmFrequency);
    // void void enableRampRate();
    // void disableRampRate();

   private:
    mbed::PwmOut *forwardPin;
    // mbed::PwmOut *reversePin;
    // unsigned short int enablePin;
    // bool rampRateEnabled;
};

#endif