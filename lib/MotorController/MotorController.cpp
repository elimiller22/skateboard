#include "MotorController.h"

#include <Arduino.h>
#include <mbed.h>

using namespace mbed;

/**
 * @brief Construct a new Motor Controller:: Motor Controller with no reverse
 * functionality or ability to put the controller to sleep.
 *
 * @param forwardPin - The forward direction pwm pin.
 */
MotorController::MotorController(unsigned short forwardPin) {
  rampRateEnabled = true;
  currentSpeed = 0;
  timeAtLastSet = 0;
  rampSteps = DEFAULT_RAMP_STEPS;
  rampDelay = DEFAULT_RAMP_DELAY_MS;
  minSpeed = MIN_SPEED;
  forwardOnly = true;
  this->forwardPin = new PwmOut(digitalPinToPinName(forwardPin));
}

/**
 * @brief Construct a new Motor Controller:: Motor Controller with forward and
 * reverse functionality but without the ability to put the controller to sleep.
 *
 * @param forwardPin - The forward direction pwm pin.
 * @param reversePin - The forward direction pwm pin.
 */
MotorController::MotorController(unsigned short forwardPin,
                                 unsigned short reversePin) {
  rampRateEnabled = true;
  rampSteps = DEFAULT_RAMP_STEPS;
  rampDelay = DEFAULT_RAMP_DELAY_MS;
  minSpeed = MIN_SPEED;
  currentSpeed = 0;
  timeAtLastSet = 0;
  forwardOnly = false;
  this->forwardPin = new PwmOut(digitalPinToPinName(forwardPin));
  this->reversePin = new PwmOut(digitalPinToPinName(reversePin));
}

/**
 * @brief Construct a new Motor Controller:: Motor Controller with forward,
 * reverse and sleep functionality to put the controller to sleep.
 *
 * @param forwardPin - The forward direction pwm pin.
 * @param reversePin - The forward direction pwm pin.
 * @param enablePin - The enable digital pin.
 */
MotorController::MotorController(unsigned short forwardPin,
                                 unsigned short reversePin,
                                 unsigned short enablePin) {
  rampRateEnabled = true;
  timeAtLastSet = 0;
  currentSpeed = 0;
  rampSteps = DEFAULT_RAMP_STEPS;
  rampDelay = DEFAULT_RAMP_DELAY_MS;
  minSpeed = MIN_SPEED;
  forwardOnly = false;
  this->forwardPin = new PwmOut(digitalPinToPinName(forwardPin));
  this->reversePin = new PwmOut(digitalPinToPinName(reversePin));
  this->enablePin = new DigitalOut(digitalPinToPinName(enablePin));
}

/**
 * @brief Get the current speed of the motor.
 *
 * @return short - The current set speed of the motor.
 */
short MotorController::getSpeed() { return currentSpeed; }

/**
 * @brief Method to set the speed Of the motor
 *
 * @param speedPercentage - The speed as a meature of percentage from -100 to
 * positive 100. With negative numbers representing reverse motor direction.
 */
void MotorController::setSpeed(short speedPercentage) {
  // If the new speed is the same as the current speed, do nothing.
  if (speedPercentage == currentSpeed)
    return; // Do nothing

  // If an applicable speed percentage is passed in, set the speed.
  if (-100 <= speedPercentage && speedPercentage <= 100) {
    if (rampRateEnabled) {
      if (timeAtLastSet == 0 || short(millis() - timeAtLastSet) >= rampDelay) {
        if (speedPercentage < currentSpeed) {
          currentSpeed -= rampSteps;
        } else if (speedPercentage > currentSpeed) {
          currentSpeed += rampSteps;
        }
        writeCurrentSpeed();
      }
    } else {
      writeCurrentSpeed();
    }

  } else {
    // If a non-applicable speed is provided, write a message to serial output,
    // and stop the motor.
    Serial.print("Improper Speed Value: Set Speed 0");
    currentSpeed = 0;
    forwardPin->write(0);
    if (!forwardOnly)
      reversePin->write(0);
    timeAtLastSet = 0;
  }
}

/**
 * @brief Private method to write the pwm speed to the pins.
 *
 */
void MotorController::writeCurrentSpeed() {
  // Print the new set speed to serial output.
  Serial.print("New Current Speed:");
  Serial.println(currentSpeed);

  // The min speed tuning factor represents the speed below which there is no
  // movement of the motor. So, tune the speed to be written accordingly so that
  // any non-zero speed produces movement.
  float writeSpeed = 0;
  if (currentSpeed != 0) {
    short diffSpeed = 100 - minSpeed;
    writeSpeed = float((abs(currentSpeed) * diffSpeed) / 100) + minSpeed;
    writeSpeed /= 100.0;
    // Print the new pwm write speed to serial output.
    Serial.print("New Write Speed:");
    Serial.println(writeSpeed);
  }

  if (currentSpeed > 0) {
    reversePin->write(0);
    forwardPin->write(writeSpeed);
    timeAtLastSet = millis();
  } else if (currentSpeed < 0 && !forwardOnly) {
    Serial.println("Write Reverse!!");
    forwardPin->write(0);
    reversePin->write(writeSpeed);
    timeAtLastSet = millis();
  } else {
    forwardPin->write(0);
    reversePin->write(0);
    timeAtLastSet = 0;
  }
}

/**
 * @brief Set the Ramp Rate in percent steps. Default is 1. Higher numbers
 * result in the motor getting to max speed quicker.
 *
 * @param rampRatePercentSteps - Steps from 0 to 100.
 */
void MotorController::setRampRate(unsigned short rampRatePercentSteps) {
  if (rampRatePercentSteps <= 0) {
    rampSteps = 0;
  } else if (rampRatePercentSteps >= 100) {
    rampSteps = 100;
  } else {
    rampSteps = rampRatePercentSteps;
  }
}

/**
 * @brief Set the Frequency for the PWM going to the motor controller.
 *
 * @param pwmFrequency - The pulse with modulation frequency.
 */
void MotorController::setFrequency(unsigned short pwmFrequency) {
  forwardPin->period(1.0 / pwmFrequency);
  reversePin->period(1.0 / pwmFrequency);
}

/**
 * @brief Method to enable ramping the motor between speeds.
 *
 */
void MotorController::enableRampRate() { rampRateEnabled = true; }

/**
 * @brief Method to disable ramping the motor to final speed.
 *
 */
void MotorController::disableRampRate() { rampRateEnabled = false; }

/**
 * @brief Set the Min Pwm Speed for the motor. This is the speed below which
 * there is no motor movement. Speed percentages are then adjusted so that
 * all non-zero speeds produce movement.
 *
 * @param speedPercentage - The minimum percentage for which speed is
 * produced, when this value is zero.
 */
void MotorController::setMinPwmSpeed(short speedPercentage) {
  minSpeed = speedPercentage;
}
