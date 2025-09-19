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


## Using Adafruit motor drivers
The [Adafruit FeatherWing](https://www.adafruit.com/product/2927) can control up to 4 DC motors and can be communicated with over I2C to set the PWM for each motor. The [Adafruit Feather RP2040](https://www.adafruit.com/product/4884) is a good choice for the microcontroller running the Arduino code interfacing with the FeatherWings.

Arduino Dependencies:
- [`Adafruit_MotorShield`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/Adafruit_MotorShield.h)
- [`Adafruit_MS_PWMServoDriver`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/utility/Adafruit_MS_PWMServoDriver.h)
- [`Adafruit_I2CDevice`](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_I2CDevice.h)
- [`Protothreads`](https://gitlab.com/airbornemint/arduino-protothreads)

Then the MotorShield class can be used along with the `Adafruit_DCMotor` class to set the PWM values for the motor, controlling the speed. For the rover, we need a single class that interacts with the motors and is able to set the PWM speeds for all 6 motors and convert RPM or rad/s (or any angular velocity) into PWM values: `RoverMotorInterface` (RMI)

`RoverMotorInterface` will need to do the following:
1. Maintain 2 instances of `Adafruit_MotorShield` (1 for each side of the rover)
2. Maintain all 6 instances of `Adafruit_DCMotor` (3 on each MotorShield)
3. Functions to safely set the target motor velocities while they are running
4. Run a process that reads a target PWM for all 6 motors and sets the PWM accordingly

A PID loop to control the target velocity will need to be implemented within the process, so the target velocity and the velocity error will need to be passed in. The velocity error will need to be derived from the encoder process running separately and then passed into the RMI's process.


[*Adafruit Feather M4 Express*](https://www.adafruit.com/product/3857) might be a better option due to better PlatformIO support and faster clock speed and better floating point performance.