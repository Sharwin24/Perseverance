# RoverMotorControl
Firmware for the Teensy 4.1 MCU on the rover, handling low-level motor control and communication with Raspberry Pi using SPI.

This board needs to interact with the following hardware:
1. Control 6 BLDC motors (6 PWM pins, 6 GPIO direction pins)
2. Control 4 servo motors (4 PWM pins)
3. Read from 6 encoders (12 GPIO pins)
4. Communicate with Raspberry Pi 5 over SPI

**RPI5 -> Teensy:**
- 6 motor speeds
- 4 servo positions (steering angles)

**Teensy -> RPI5:**
- 6 encoder positions (for wheel odometry)

**Individually Threaded Tasks**
1. Encoder Task: Reading Encoders and saving encoder count (1kHz)
2. Motor Task: PID Control loop for drive motors and updating PWM signals (200Hz)
3. Communication Task: Handle SPI communication with RPI5 (100Hz)
4. Steering Task: Control steering servos based on RPI5 commands (100Hz)