#ifndef ROVER_PINS_H
#define ROVER_PINS_H

// Pin Map for Rover Control
// Rationale:
//  - Keep default SPI (MOSI=11, MISO=12, SCK=13) free; only CS defined explicitly
//  - Reserve pin 13 for onboard LED only
//  - Use primary I2C bus SDA=18, SCL=19
//  - Group each wheel's encoder pins adjacently where possible

// Front Left (FL)
#define FL_ENCODER_A_PIN 5
#define FL_ENCODER_B_PIN 6
#define FL_STEERING_SERVO_PIN 7

// Front Right (FR)
#define FR_ENCODER_A_PIN 14
#define FR_ENCODER_B_PIN 15
#define FR_STEERING_SERVO_PIN 8

// Middle Left (ML)
#define ML_ENCODER_A_PIN 23
#define ML_ENCODER_B_PIN 24

// Middle Right (MR)
#define MR_ENCODER_A_PIN 27
#define MR_ENCODER_B_PIN 28

// Rear Left (RL)
#define RL_ENCODER_A_PIN 31
#define RL_ENCODER_B_PIN 32
#define RL_STEERING_SERVO_PIN 33

// Rear Right (RR)
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

// GPIO pin to signal low battery condition and enable charging circuit to operate
#define BATTERY_CHARGE_ENABLE_PIN 38

// Sanity compile-time checks (optional - enable if desired)
// static_assert(DEBUG_LED != FR_STEERING_SERVO_PIN, "LED and FR servo share a pin");

#endif // !ROVER_PINS_H
