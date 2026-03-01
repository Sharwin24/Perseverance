# RoverFirmware
Firmware for the [**Adafruit Feather M4 Express**](https://www.adafruit.com/product/3857) (SAMD51, Cortex-M4F) on the rover, handling low-level motor control and SPI communication with the Raspberry Pi 5.

## Why the Feather M4 Express?

The M4 Express was chosen over alternatives (RP2040, Teensy 4.1) for the following reasons:

- **Hardware FPU:** The SAMD51 has a single-precision hardware FPU, making the PID loops across 6 motors at 100Hz fast and deterministic. The RP2040 has no hardware FPU, so all floating-point math runs in software emulation.
- **Interrupt-capable GPIOs:** Every digital pin supports external interrupts, covering all 12 pins needed for 6 quadrature encoders without workarounds.
- **Adafruit library compatibility:** First-class support for `Adafruit_MotorShield`, `Adafruit_BusIO`, and `Adafruit_MAX1704X`. The RP2040's community Arduino core has occasional compatibility quirks with these libraries.
- **PlatformIO support:** The M4 has native PlatformIO support. The RP2040 requires the community `maxgerhardt/platform-raspberrypi` platform and needed workarounds (e.g., `-DENCODER_DO_NOT_USE_INTERRUPTS`) that would defeat high-frequency encoder reading.

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

All tasks run cooperatively using `ArduinoThread`, checked in priority order in `loop()`:

| Priority | Task | Rate | Description |
|----------|------|------|-------------|
| 1 | Encoder Task | 1 kHz | Reads 6 quadrature encoders and updates encoder counts |
| 2 | Motor Task | 100 Hz | PID velocity control for 6 DC motors, updates FeatherWing PWM |
| 3 | Steering Task | 100 Hz | Updates 4 steering servos based on RPi5 commands |
| 4 | Communication Task | 100 Hz | SPI — receives commands from RPi5, sends back encoder data |
| 5 | Battery Monitor Task | 0.5 Hz | Polls MAX17048 alerts and controls charging circuit |
| 6 | Heartbeat Task | 0.25 Hz | Pulses built-in LED to indicate scheduler is alive |

## Arduino / PlatformIO Dependencies

- [`Adafruit_MotorShield`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/Adafruit_MotorShield.h)
- [`Adafruit_MS_PWMServoDriver`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/utility/Adafruit_MS_PWMServoDriver.h)
- [`Adafruit_I2CDevice`](https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_I2CDevice.h)
- [`Adafruit_MAX1704X`](https://github.com/adafruit/Adafruit_MAX1704X) — battery fuel gauge
- [`ArduinoThread`](https://gitlab.com/airbornemint/arduino-protothreads) — cooperative multitasking

## Task Details

### Encoder Task — 1 kHz

Reads all 6 quadrature encoders using ISR-driven `Encoder` objects and stores the raw tick counts into the shared `encoderCounts[6]` array (`{FL, FR, ML, MR, RL, RR}`). Running at 1 kHz gives 1 ms resolution for the velocity estimation performed in the Motor Task.

### Motor Task — 100 Hz

Implements an independent PID velocity control loop for each of the 6 drive motors:

1. Computes `dt` in microseconds via `micros()` for precise, wrap-safe timing.
2. Derives current velocity for each motor as `(current_ticks - prev_ticks) / dt` in ticks/s.
3. Sets each PID controller's input to the measured velocity and setpoint to the commanded speed from `motorSpeedCommands[6]` (rad/s, populated by the Communication Task).
4. Maps the PID output (clamped to ±4095, 12-bit range) to `setSpeedFine()` on each `Adafruit_DCMotor`, with direction set by the sign of the output.

PID gains: `Kp = 2.0`, `Ki = 5.0`, `Kd = 1.0`. The two MotorShields sit at I2C addresses `0x60` (left side: FL, ML, RL) and `0x61` (right side: FR, MR, RR).

### Steering Task — 100 Hz

Reads the shared `servoAngleCommands[4]` array (`{FL, FR, RL, RR}` in radians) and writes positions to the 4 steering servos. Angles are converted from radians to degrees, then mapped from `[-90°, 90°]` to the `[0°, 180°]` servo range.

### Communication Task — 100 Hz

Handles the full-duplex SPI transaction with the Raspberry Pi 5:

- **TX (24 bytes):** 6 encoder counts packed as little-endian `uint32_t` values.
- **RX (40 bytes):** 6 motor speed `float`s (bytes 0–23) followed by 4 servo angle `float`s (bytes 24–39), all little-endian.

Values are unpacked via a `union { uint32_t u; float f; }` bitcast and written into `motorSpeedCommands` and `servoAngleCommands`. On a failed SPI transaction, previous command values are retained unchanged.

### Battery Monitor Task — 0.5 Hz

Polls the MAX17048 fuel gauge for active alerts and responds accordingly:

| Alert Flag | Action |
|---|---|
| `SOC_CHANGE` | Clear flag |
| `SOC_LOW` | Clear flag |
| `VOLTAGE_RESET` | Clear flag |
| `VOLTAGE_LOW` (< 3.25 V) | Assert `BATTERY_CHARGE_ENABLE_PIN` HIGH — enables charging circuit |
| `VOLTAGE_HIGH` (> 4.25 V) | Assert `BATTERY_CHARGE_ENABLE_PIN` LOW — disables charging circuit |
| `RESET_INDICATOR` | Clear flag |

Thresholds: low = 3.30 V, full = 4.20 V, hysteresis = 0.05 V (prevents rapid toggling at boundary voltages).

### Heartbeat Task — 0.25 Hz

Pulses the built-in LED for 100 ms every 4 seconds as a visual indicator that the cooperative scheduler is alive and not deadlocked.

---

## Motor Interface

`RoverMotorInterface` (RMI) wraps the two `Adafruit_MotorShield` instances and all 6 `Adafruit_DCMotor` objects. It:

1. Maintains 2 `Adafruit_MotorShield` instances (one per side of the rover)
2. Manages 6 `Adafruit_DCMotor` instances (3 per shield)
3. Exposes thread-safe setters for target velocities
4. Runs a PID control loop, reading encoder-derived velocity error and updating PWM output each Motor Task cycle
