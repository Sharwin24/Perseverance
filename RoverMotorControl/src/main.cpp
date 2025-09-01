#include <Arduino.h>

/**
 * This board (Teensy) needs to perform the following tasks:
 * 1. Control 6 BLDC motors (6 PWM pins, 6 GPIO direction pins)
 * 2. Control 4 servo motors (4 PWM pins)
 * 3. Read from 6 encoders (12 GPIO pins)
 * 4. Communicate with Raspberry Pi 5 over SPI to send encoder data and receive motor commands
 *
 * RPI5 -> Teensy:
 * - 6 motor speeds
 * - 4 servo positions (steering angles)
 * Teensy -> RPI5:
 * - 6 encoder positions (for wheel odometry)
 *
 */

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}