#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include "rover_pins.h"

class WheelData {
public:
  static constexpr uint8_t numWheels = 6;
  static constexpr uint8_t numSteering = 4; // FL, FR, RL, RR

  WheelData();
  ~WheelData();

  // Read all encoder counts into encoder_positions[]
  void readEncoders();

  // Reset all encoder counts to zero
  void resetEncoders();

  // Public pin arrays for higher-level control modules
  uint8_t motor_pwm_pins[numWheels];
  uint8_t motor_dir_pins[numWheels];
  uint8_t steering_servo_pins[numSteering];
  int32_t encoder_positions[numWheels];

  // Access a specific encoder count (no bounds check)
  inline int32_t getEncoder(uint8_t idx) const { return encoder_positions[idx]; }

private:
  Encoder encoders[numWheels];
  Servo servos[numSteering];
};
