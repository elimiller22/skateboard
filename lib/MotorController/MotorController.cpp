#include "MotorController.h"

#include <Arduino.h>
#include <mbed.h>

// #include "pinDefinitions.h"

using namespace mbed;

MotorController::MotorController(unsigned short forwardPin) {
    rampRateEnabled = true;
    currentSpeed = 0;
    timeAtLastSet = 0;
    rampSteps = defaultRampSteps;
    rampDelay = defaultRampDelay_ms;
    this->forwardPin = new PwmOut(digitalPinToPinName(forwardPin));
}

MotorController::MotorController(unsigned short forwardPin, unsigned short reversePin) {
    rampRateEnabled = true;
    rampSteps = defaultRampSteps;
    rampDelay = defaultRampDelay_ms;
    currentSpeed = 0;
    timeAtLastSet = 0;
    this->forwardPin = new PwmOut(digitalPinToPinName(forwardPin));
    this->reversePin = new PwmOut(digitalPinToPinName(reversePin));
}

MotorController::MotorController(
    unsigned short forwardPin, unsigned short reversePin, unsigned short enablePin) {
    rampRateEnabled = true;
    timeAtLastSet = 0;
    currentSpeed = 0;
    rampSteps = defaultRampSteps;
    rampDelay = defaultRampDelay_ms;
    this->forwardPin = new PwmOut(digitalPinToPinName(forwardPin));
    this->reversePin = new PwmOut(digitalPinToPinName(reversePin));
    this->enablePin = new DigitalOut(digitalPinToPinName(enablePin));
}

float MotorController::getSpeed() {
    return currentSpeed;
}

void MotorController::setSpeed(short speedPercentage) {
    if (-100 <= speedPercentage && speedPercentage <= 100) {
        // float newSpeed = speedPercentage / 100.0;
        // float setSpeed = newSpeed;
        if (timeAtLastSet == 0 || short(millis() - timeAtLastSet) >= rampDelay) {
            if (speedPercentage < currentSpeed) {
                currentSpeed -= rampSteps;
            } else if (speedPercentage > currentSpeed) {
                currentSpeed += rampSteps;
            }
            Serial.print("New Current Speed:");
            Serial.println(currentSpeed);
            if (currentSpeed > 0) {
                reversePin->write(0);
                forwardPin->write(currentSpeed / 100.0);
                timeAtLastSet = millis();
            } else if (currentSpeed < 0) {
                Serial.println("Write Reverse!!");
                forwardPin->write(0);
                reversePin->write(abs(currentSpeed) / 100.0);
                timeAtLastSet = millis();
            } else {
                forwardPin->write(0);
                reversePin->write(0);
                timeAtLastSet = 0;
            }
        }
    } else {
        Serial.print("Improper Speed Value: Set Speed 0");
        currentSpeed = 0;
        forwardPin->write(0);
        timeAtLastSet = 0;
    }

    // if (0 < speedPercentage && speedPercentage <= 100) {
    //     float speed = speedPercentage / 100.0;
    //     Serial.print("Speed set to: ");
    //     Serial.println(speed);
    //     forwardPin->write(speed);
    // } else {
    //     forwardPin->write(0);
    // }
}

// void MotorController::setRampRate(float rampRatePerSecond) {
//     if (rampRatePerSecond <= 0.01) {
//         rampRate = 0.01;
//     } else {
//         rampRate = rampRatePerSecond;
//     }
// }

void MotorController::setFrequency(unsigned short pwmFrequency) {
    forwardPin->period(1.0 / pwmFrequency);
    reversePin->period(1.0 / pwmFrequency);
}

void MotorController::enableRampRate() {
    rampRateEnabled = true;
}

void MotorController::disableRampRate() {
    rampRateEnabled = false;
}
