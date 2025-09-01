#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Encoder.h>
#include "rover_pins.h"
#include "wheel_data.h"

/**
 * This board (Teensy 4.1) needs to interact with the following hardware:
 * 1. Control 6 BLDC motors (6 PWM pins, 6 GPIO direction pins)
 * 2. Control 4 servo motors (4 PWM pins)
 * 3. Read from 6 encoders (12 GPIO pins)
 * 4. Communicate with Raspberry Pi 5 over SPI
 *
 * RPI5 -> Teensy (SPI):
 * - 6 motor speeds
 * - 4 servo positions (steering angles)
 * Teensy -> RPI5 (SPI):
 * - 6 encoder positions (for wheel odometry)
 *
 */

 // Global wheel data instance
WheelData g_wheelData;

void setup() {
  pinMode(DEBUG_LED, OUTPUT);
}

void loop() {
  g_wheelData.readEncoders();
  // Example: blink LED every second based on encoder sum (placeholder logic)
  static elapsedMillis sinceBlink;
  if (sinceBlink > 1000) {
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
    sinceBlink = 0;
  }
}