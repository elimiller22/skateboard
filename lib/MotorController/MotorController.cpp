#include "MotorController.h"

#include <Arduino.h>

#include "mbed.h"
#include "pinDefinitions.h"

MotorController::MotorController(unsigned short forwardPin) {
    this->forwardPin = new mbed::PwmOut(digitalPinToPinName(forwardPin));
}

void MotorController::setFrequency(unsigned short pwmFrequency) {
    forwardPin->period(1.0 / pwmFrequency);
}

void MotorController::setSpeed(unsigned short speedPercentage) {
    if (0 < speedPercentage && speedPercentage <= 100) {
        float speed = speedPercentage / 100.0;
        Serial.print("Speed set to: ");
        Serial.println(speed);
        forwardPin->write(speed);
    } else {
        forwardPin->write(0);
    }
}