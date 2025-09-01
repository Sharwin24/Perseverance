// wheel_data.cpp - implementation of WheelData
#include "wheel_data.h"

WheelData::WheelData()
  : encoders{
      Encoder(FL_ENCODER_A_PIN, FL_ENCODER_B_PIN),
      Encoder(FR_ENCODER_A_PIN, FR_ENCODER_B_PIN),
      Encoder(ML_ENCODER_A_PIN, ML_ENCODER_B_PIN),
      Encoder(MR_ENCODER_A_PIN, MR_ENCODER_B_PIN),
      Encoder(RL_ENCODER_A_PIN, RL_ENCODER_B_PIN),
      Encoder(RR_ENCODER_A_PIN, RR_ENCODER_B_PIN)},
      servos{
        Servo(), Servo(), Servo(), Servo()
      } {

  // Motor PWM pins
  motor_pwm_pins[0] = FL_MOTOR_PWM_PIN;
  motor_pwm_pins[1] = FR_MOTOR_PWM_PIN;
  motor_pwm_pins[2] = ML_MOTOR_PWM_PIN;
  motor_pwm_pins[3] = MR_MOTOR_PWM_PIN;
  motor_pwm_pins[4] = RL_MOTOR_PWM_PIN;
  motor_pwm_pins[5] = RR_MOTOR_PWM_PIN;

  // Motor direction pins
  motor_dir_pins[0] = FL_MOTOR_DIR_PIN;
  motor_dir_pins[1] = FR_MOTOR_DIR_PIN;
  motor_dir_pins[2] = ML_MOTOR_DIR_PIN;
  motor_dir_pins[3] = MR_MOTOR_DIR_PIN;
  motor_dir_pins[4] = RL_MOTOR_DIR_PIN;
  motor_dir_pins[5] = RR_MOTOR_DIR_PIN;

  // Steering servo pins (indices: FL, FR, RL, RR)
  steering_servo_pins[0] = FL_STEERING_SERVO_PIN;
  steering_servo_pins[1] = FR_STEERING_SERVO_PIN;
  steering_servo_pins[2] = RL_STEERING_SERVO_PIN;
  steering_servo_pins[3] = RR_STEERING_SERVO_PIN;
  // Attach servos to their pins
  for (uint8_t i = 0; i < numSteering; ++i) {
    servos[i].attach(steering_servo_pins[i]);
  }

  // Init encoder positions
  for (uint8_t i = 0; i < numWheels; ++i) {
    encoder_positions[i] = 0;
  }

  // Initialize motor direction pins
  for (uint8_t i = 0; i < numWheels; ++i) {
    pinMode(motor_dir_pins[i], OUTPUT);
    pinMode(motor_pwm_pins[i], OUTPUT);
  }
}

WheelData::~WheelData() {
  // Destructor
}

void WheelData::readEncoders() {
  for (uint8_t i = 0; i < numWheels; ++i) {
    encoder_positions[i] = encoders[i].read();
  }
}

void WheelData::resetEncoders() {
  for (uint8_t i = 0; i < numWheels; ++i) {
    encoders[i].readAndReset();
    encoder_positions[i] = 0;
  }
}
