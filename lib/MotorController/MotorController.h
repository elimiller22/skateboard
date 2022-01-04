#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include <mbed.h>

using namespace mbed;

// Default tuning values constants for motor control.
const short DEFAULT_RAMP_STEPS = 1;
const short DEFAULT_RAMP_DELAY_MS = 5;
const short MIN_SPEED = 0;

/**
 * @brief Motorcontroller class. Provides functionality to control the
 * skateboard speed and direction.
 *
 */
class MotorController {
public:
  /**
   * @brief Construct a new Motor Controller:: Motor Controller with no reverse
   * functionality or ability to put the controller to sleep.
   *
   * @param forwardPin - The forward direction pwm pin.
   */
  MotorController(unsigned short forwardPin);

  /**
   * @brief Construct a new Motor Controller:: Motor Controller with forward and
   * reverse functionality but without the ability to put the controller to
   * sleep.
   *
   * @param forwardPin - The forward direction pwm pin.
   * @param reversePin - The forward direction pwm pin.
   */
  MotorController(unsigned short forwardPin, unsigned short reversePin);

  /**
   * @brief Construct a new Motor Controller:: Motor Controller with forward,
   * reverse and sleep functionality to put the controller to sleep.
   *
   * @param forwardPin - The forward direction pwm pin.
   * @param reversePin - The forward direction pwm pin.
   * @param enablePin - The enable digital pin.
   */
  MotorController(
      unsigned short forwardPin, unsigned short reversePin,
      unsigned short enablePin); // todo - Add enablePin functionality

  /**
   * @brief Set the Min Pwm Speed for the motor. This is the speed below which
   * there is no motor movement. Speed percentages are then adjusted so that
   * all non-zero speeds produce movement.
   *
   * @param speedPercentage - The minimum percentage for which speed is
   * produced, when this value is zero.
   */
  void setMinPwmSpeed(short speedPercentage);

  /**
   * @brief Method to set the speed Of the motor
   *
   * @param speedPercentage - The speed as a meature of percentage from -100 to
   * positive 100. With negative numbers representing reverse motor direction.
   */
  void setSpeed(short speedPercentage);

  /**
   * @brief Get the current speed of the motor.
   *
   * @return short - The current set speed of the motor.
   */
  short getSpeed();

  /**
   * @brief Set the Ramp Rate in percent steps. Default is 1. Higher numbers
   * result in the motor getting to max speed quicker.
   *
   * @param rampRatePercentSteps - Steps from 0 to 100.
   */
  void setRampRate(unsigned short rampRatePercentSteps);

  /**
   * @brief Set the Frequency for the PWM going to the motor controller.
   *
   * @param pwmFrequency - The pulse with modulation frequency.
   */
  void setFrequency(unsigned short pwmFrequency);

  /**
   * @brief Method to enable ramping the motor between speeds.
   *
   */
  void enableRampRate();

  /**
   * @brief Method to disable ramping the motor to final speed.
   *
   */
  void disableRampRate();

private: // Define private variables and functions.
  PwmOut *forwardPin;
  PwmOut *reversePin;
  DigitalOut *enablePin;
  bool rampRateEnabled;
  bool forwardOnly;
  unsigned short rampSteps;
  unsigned short rampDelay;
  unsigned long timeAtLastSet;
  short currentSpeed;
  short minSpeed;
  void writeCurrentSpeed();
};

#endif