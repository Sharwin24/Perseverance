# RoverFirmware
Firmware for the **Adafruit Feather M4 Express** (SAMD51, Cortex-M4F) on the rover, handling low-level motor control and SPI communication with the Raspberry Pi 5.

## Why the Feather M4 Express?

The M4 Express was chosen over alternatives (RP2040, Teensy 4.1) for the following reasons:

- **Hardware FPU:** The SAMD51 has a single-precision hardware FPU, making the PID loops across 6 motors at 100Hz fast and deterministic. The RP2040 has no hardware FPU, so all floating-point math runs in software emulation.
- **Interrupt-capable GPIOs:** Every digital pin supports external interrupts, covering all 12 pins needed for 6 quadrature encoders without workarounds.
- **Adafruit library compatibility:** First-class support for `Adafruit_MotorShield`, `Adafruit_BusIO`, and `Adafruit_MAX1704X`. The RP2040's community Arduino core has occasional compatibility quirks with these libraries.
- **PlatformIO support:** The M4 has native PlatformIO support. The RP2040 requires the community `maxgerhardt/platform-raspberrypi` platform and needed workarounds (e.g., `-DENCODER_DO_NOT_USE_INTERRUPTS`) that would defeat high-frequency encoder reading.
- **Stemma QT connector:** Makes daisy-chaining the two MotorShield FeatherWings and the MAX17048 battery monitor straightforward.

## Hardware Interfaces

1. Control 6 DC drive motors via 2× Adafruit Motor Shield FeatherWings (I2C)
2. Control 4 steering servos (PWM)
3. Read from 6 quadrature encoders (12 interrupt pins)
4. Communicate with Raspberry Pi 5 over SPI

## SPI Protocol

**RPi5 → M4:**
- 6 motor target speeds
- 4 servo steering angles

**M4 → RPi5:**
- 6 encoder counts (for wheel odometry)

## Concurrent Tasks

All tasks run cooperatively using `ArduinoThread`:

| Task | Rate | Description |
|------|------|-------------|
| Encoder Task | 1 kHz | Reads 6 quadrature encoders and updates encoder counts |
| Motor Task | 100 Hz | PID velocity control for 6 DC motors, updates FeatherWing PWM |
| Steering Task | 100 Hz | Updates 4 steering servos based on RPi5 commands |
| Communication Task | 100 Hz | SPI slave — receives commands, sends back encoder data |

## Arduino / PlatformIO Dependencies

- [`Adafruit_MotorShield`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/Adafruit_MotorShield.h)
- [`Adafruit_MS_PWMServoDriver`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/utility/Adafruit_MS_PWMServoDriver.h)
- [`Adafruit_I2CDevice`](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_I2CDevice.h)
- [`Adafruit_MAX1704X`](https://github.com/adafruit/Adafruit_MAX1704X) — battery fuel gauge
- [`ArduinoThread`](https://gitlab.com/airbornemint/arduino-protothreads) — cooperative multitasking

## Motor Interface

`RoverMotorInterface` (RMI) wraps the two `Adafruit_MotorShield` instances and all 6 `Adafruit_DCMotor` objects. It:

1. Maintains 2 `Adafruit_MotorShield` instances (one per side of the rover)
2. Manages 6 `Adafruit_DCMotor` instances (3 per shield)
3. Exposes thread-safe setters for target velocities
4. Runs a PID control loop, reading encoder-derived velocity error and updating PWM output each Motor Task cycle
