#ifndef ROVER_PINS_H
#define ROVER_PINS_H

// Pin Map for Rover Control
// Rationale:
//  - Keep default SPI (MOSI=11, MISO=12, SCK=13) free; only CS defined explicitly
//  - Reserve pin 13 for onboard LED only
//  - Use primary I2C bus SDA=18, SCL=19
//  - Group each wheel's encoder pins adjacently where possible

// Onboard Orange LED
#define DEBUG_LED 13

// Front Left (FL)
#define FL_MOTOR_PWM_PIN 3
#define FL_MOTOR_DIR_PIN 4
#define FL_ENCODER_A_PIN 5
#define FL_ENCODER_B_PIN 6
#define FL_STEERING_SERVO_PIN 7

// Front Right (FR)
#define FR_MOTOR_PWM_PIN 9
#define FR_MOTOR_DIR_PIN 20
#define FR_ENCODER_A_PIN 14
#define FR_ENCODER_B_PIN 15
#define FR_STEERING_SERVO_PIN 8

// Middle Left (ML)
#define ML_MOTOR_PWM_PIN 21
#define ML_MOTOR_DIR_PIN 22
#define ML_ENCODER_A_PIN 23
#define ML_ENCODER_B_PIN 24

// Middle Right (MR)
#define MR_MOTOR_PWM_PIN 25
#define MR_MOTOR_DIR_PIN 26
#define MR_ENCODER_A_PIN 27
#define MR_ENCODER_B_PIN 28

// Rear Left (RL)
#define RL_MOTOR_PWM_PIN 29
#define RL_MOTOR_DIR_PIN 30
#define RL_ENCODER_A_PIN 31
#define RL_ENCODER_B_PIN 32
#define RL_STEERING_SERVO_PIN 33

// Rear Right (RR) - bottom pads
#define RR_MOTOR_PWM_PIN 2
#define RR_MOTOR_DIR_PIN 34
#define RR_ENCODER_A_PIN 35
#define RR_ENCODER_B_PIN 36
#define RR_STEERING_SERVO_PIN 37

// SPI Pins
#define SPI_CS_PIN 10
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 12
#define SPI_SCK_PIN 13

// RPI5 Pins
// MOSI = GPIO10(phys 19)
// MISO = GPIO9(phys 21)
// SCLK = GPIO11(phys 23)
// CE0 = GPIO8(phys 24)

// I2C (primary Wire bus)
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

// Sanity compile-time checks (optional - enable if desired)
// static_assert(DEBUG_LED != FR_STEERING_SERVO_PIN, "LED and FR servo share a pin");

#endif // !ROVER_PINS_H
