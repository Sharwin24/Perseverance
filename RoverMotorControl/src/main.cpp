/**
 * @file main.cpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Main control code for the Perseverance Rover's motor and servo control using Feather RP2040
 * @date 2024-09-016
 * @version 2.0
 *
 * This board needs to interact with the following hardware:
 * 1. Control 6 BLDC motors (6 PWM pins, 6 GPIO direction pins)
 * 2. Control 4 servo motors (4 PWM pins)
 * 3. Read from 6 encoders (12 GPIO pins)
 * 4. Communicate with Raspberry Pi 5 over SPI
 *
 * RPI5 -> MCU (SPI):
 * - 6 motor speeds
 * - 4 servo positions (steering angles)
 * MCU -> RPI5 (SPI):
 * - 6 encoder positions (for wheel odometry)
 *
 * 1. Encoder Task: Reading Encoders and saving encoder count (1kHz)
 * 2. Motor Task: PID Control loop for drive motors and updating PWM signals (200Hz)
 * 3. Communication Task: Handle SPI communication with RPI5 (100Hz)
 * 4. Steering Task: Control steering servos based on RPI5 commands (100Hz)
 */


#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Thread.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_SPIDevice.h>

#include "rover_pins.h"

#define SERIAL_BAUD 115200 // Baud rate for Serial communication
#define PWM_FREQ 20000 // 20 kHz PWM frequency for motor control

 // --- Task Frequencies ---
#define MOTOR_TASK_FREQ 200 // [Hz]
#define MOTOR_TASK_PERIOD_MS (1000 / MOTOR_TASK_FREQ)
#define ENCODER_TASK_FREQ 1000 // [Hz]
#define ENCODER_TASK_PERIOD_MS (1000 / ENCODER_TASK_FREQ)
#define COMM_TASK_FREQ 100 // [Hz]
#define COMM_TASK_PERIOD_MS (1000 / COMM_TASK_FREQ)
#define STEER_TASK_FREQ 100 // [Hz]
#define STEER_TASK_PERIOD_MS (1000 / STEER_TASK_FREQ)
#define HEARTBEAT_TASK_FREQ 1 // [Hz]
#define HEARTBEAT_TASK_PERIOD_MS (1000 / HEARTBEAT_TASK_FREQ)

// Motor Shield with I2C addresses 0x60 and 0x61 hosting the left and right 3 drive motors respectively
Adafruit_MotorShield AFMSLeft = Adafruit_MotorShield(0x60);
Adafruit_MotorShield AFMSRight = Adafruit_MotorShield(0x61);

// Create motor objects
Adafruit_DCMotor* FLMotor = AFMSLeft.getMotor(1);
Adafruit_DCMotor* MLMotor = AFMSLeft.getMotor(2);
Adafruit_DCMotor* RLMotor = AFMSLeft.getMotor(3);
Adafruit_DCMotor* FRMotor = AFMSRight.getMotor(1);
Adafruit_DCMotor* MRMotor = AFMSRight.getMotor(2);
Adafruit_DCMotor* RRMotor = AFMSRight.getMotor(3);

// Array of pointers to each motor that aligns with command and encoder arrays
Adafruit_DCMotor* motors[6] = {FLMotor, FRMotor, MLMotor, MRMotor, RLMotor, RRMotor};


// Threads
Thread motorThread = Thread();
Thread encoderThread = Thread();
Thread steeringThread = Thread();
Thread commsThread = Thread();
Thread heartbeatThread = Thread();

// --- Shared Data and Mutexes ---
// Use volatile for variables shared between threads
volatile float motorSpeedCommands[6] = {0, 0, 0, 0, 0, 0}; // {FL, FR, ML, MR, RL, RR} [rad/s]
volatile float servoAngleCommands[4] = {0, 0, 0, 0}; // {FL, FR, RL, RR} [radians]
volatile long encoderCounts[6] = {0, 0, 0, 0, 0, 0}; // {FL, FR, ML, MR, RL, RR} [ticks]
bool resetEncoders = false; // Flag to reset encoders

// Global timing variables for tasks
unsigned long prevMotorUpdate = 0;
long prevEncoderCounts[6] = {0, 0, 0, 0, 0, 0};

// SPI Device for communication with RPI5
Adafruit_SPIDevice spiDevice(SPI_CS_PIN, SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

// --- Peripherals and Controllers ---
// PID control instances
const double Kp = 2.0, Ki = 5.0, Kd = 1.0;
double pid_inputs[6], pid_outputs[6], pid_targets[6];

PID pid_controllers[6] = {
    PID(&pid_inputs[0], &pid_outputs[0], &pid_targets[0], Kp, Ki, Kd, DIRECT),
    PID(&pid_inputs[1], &pid_outputs[1], &pid_targets[1], Kp, Ki, Kd, DIRECT),
    PID(&pid_inputs[2], &pid_outputs[2], &pid_targets[2], Kp, Ki, Kd, DIRECT),
    PID(&pid_inputs[3], &pid_outputs[3], &pid_targets[3], Kp, Ki, Kd, DIRECT),
    PID(&pid_inputs[4], &pid_outputs[4], &pid_targets[4], Kp, Ki, Kd, DIRECT),
    PID(&pid_inputs[5], &pid_outputs[5], &pid_targets[5], Kp, Ki, Kd, DIRECT),
};

// Encoder instances
Encoder encoders[6] = {
    Encoder(FL_ENCODER_A_PIN, FL_ENCODER_B_PIN),
    Encoder(FR_ENCODER_A_PIN, FR_ENCODER_B_PIN),
    Encoder(ML_ENCODER_A_PIN, ML_ENCODER_B_PIN),
    Encoder(MR_ENCODER_A_PIN, MR_ENCODER_B_PIN),
    Encoder(RL_ENCODER_A_PIN, RL_ENCODER_B_PIN),
    Encoder(RR_ENCODER_A_PIN, RR_ENCODER_B_PIN),
};

// Servo instances
Servo servos[4];
const uint8_t servo_pins[4] = {
    FL_STEERING_SERVO_PIN,
    FR_STEERING_SERVO_PIN,
    RL_STEERING_SERVO_PIN,
    RR_STEERING_SERVO_PIN,
};

// --- Task Functions ---


/**
 * @brief Encoder task running at 1kHz
 *
 * @note Stores latest encoder values from hardware encoders (ISR driven)
 * into shared encoderCounts array which will be communicated to RPI5
 * and used for PID feedback
 *
 */
void encoderTask() {
  for (uint8_t i = 0; i < 6; ++i) {
    if (resetEncoders) {
      encoderCounts[i] = encoders[i].readAndReset();
      resetEncoders = false;
    }
    else {
      encoderCounts[i] = encoders[i].read();
    }
  }
}

/**
 * @brief Motor control task running at 200Hz
 *
 * @note Implements a PID control loop for each drive motor
 * using the encoder feedback to calculate motor velocities
 *
 */
void motorTask() {
  unsigned long now = millis();
  // Calculate delta time between calculations [sec]
  double dt = (now - prevMotorUpdate) / 1000.0;
  prevMotorUpdate = now;

  // Save a local copy of the motor speed commands to minimize time spent
  // accessing the shared volatile array
  double local_motor_cmd[6];
  for (uint8_t i = 0; i < 6; ++i) {
    local_motor_cmd[i] = motorSpeedCommands[i];
  }

  for (uint8_t i = 0; i < 6; ++i) {
    long c = encoderCounts[i];
    double vel = (double)(c - prevEncoderCounts[i]) / dt; // ticks/s
    prevEncoderCounts[i] = c;

    pid_inputs[i] = vel;
    pid_targets[i] = local_motor_cmd[i];
    pid_controllers[i].Compute();
    double output = pid_outputs[i];
    uint16_t pwm = static_cast<uint16_t>(fabs(output));
    motors[i]->setSpeedFine(pwm);
    motors[i]->run((output >= 0) ? FORWARD : BACKWARD);
  }
}


/**
 * @brief Steering task running at 100Hz
 *
 * @note Accesses shared servoAngleCommands array to set servo positions
 *
 */
void steeringTask() {
  for (uint8_t i = 0; i < 4; i++) {
    // Convert radians to degrees for Servo library
    float degrees = degrees(servoAngleCommands[i]);
    servos[i].write(map(degrees, -90, 90, 0, 180));
  }
}

/**
 * @brief Communication task running at 100Hz
 *
 * @note Handles SPI communication with RPI5 and accesses shared command arrays
 *
 */
void commsTask() {
  // Prepare transmission buffer with encoder counts (6 x 4 bytes = 24 bytes)
  uint8_t txBuffer[24];
  uint8_t rxBuffer[40]; // Receive buffer for motor commands (6x4) + servo commands (4x4) = 40 bytes

  // Pack encoder counts into transmission buffer as little-endian 32-bit values
  for (uint8_t i = 0; i < 6; ++i) {
    uint32_t count = (uint32_t)encoderCounts[i];
    txBuffer[i * 4 + 0] = (count >> 0) & 0xFF;
    txBuffer[i * 4 + 1] = (count >> 8) & 0xFF;
    txBuffer[i * 4 + 2] = (count >> 16) & 0xFF;
    txBuffer[i * 4 + 3] = (count >> 24) & 0xFF;
  }

  // Perform SPI transaction: send encoder data and receive commands
  if (spiDevice.write_then_read(txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer))) {

    // Unpack received motor speed commands (6 floats)
    for (int i = 0; i < 6; ++i) {
      union { uint32_t u; float f; } u32f;
      u32f.u = ((uint32_t)rxBuffer[i * 4 + 0] << 0) |
        ((uint32_t)rxBuffer[i * 4 + 1] << 8) |
        ((uint32_t)rxBuffer[i * 4 + 2] << 16) |
        ((uint32_t)rxBuffer[i * 4 + 3] << 24);
      motorSpeedCommands[i] = u32f.f;
    }

    // Unpack received servo angle commands (4 floats)
    for (int i = 0; i < 4; ++i) {
      union { uint32_t u; float f; } u32f;
      uint8_t* ptr = &rxBuffer[24 + i * 4]; // Start after motor commands (6*4=24 bytes)
      u32f.u = ((uint32_t)ptr[0] << 0) |
        ((uint32_t)ptr[1] << 8) |
        ((uint32_t)ptr[2] << 16) |
        ((uint32_t)ptr[3] << 24);
      servoAngleCommands[i] = u32f.f;
    }
  }
  // If SPI transaction fails, commands remain at their previous values
}

/**
 * @brief Heartbeat task running at 1Hz
 *
 * @note Indicates system is running
 *
 */
void heartbeatTask() {
  digitalWrite(DEBUG_LED, HIGH);
  delay(100);
  digitalWrite(DEBUG_LED, LOW);
  delay(900);
}


void setup() {
  // Initialize Serial port
  Serial.begin(SERIAL_BAUD);
  pinMode(DEBUG_LED, OUTPUT);

  // Attach servo motors
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(servo_pins[i], OUTPUT);
    servos[i].attach(servo_pins[i]);
  }

  // Set up PID controllers
  for (uint8_t i = 0; i < 6; ++i) {
    pid_controllers[i].SetMode(AUTOMATIC);
    pid_controllers[i].SetOutputLimits(-4095, 4085); // Use 12-bit PWM
    pid_controllers[i].SetSampleTime(MOTOR_TASK_PERIOD_MS); // = 5 ms for 200 Hz
  }

  // Initialize Motor Shields
  AFMSLeft.begin();
  AFMSRight.begin();

  // Enable motors
  for (uint8_t i = 0; i < 6; i++) {
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }

  // Reset encoders on startup
  resetEncoders = true;

  // Initialize SPI
  spiDevice.begin();

  // Attach callbacks and set intervals for each thread
  motorThread.onRun(motorTask);
  encoderThread.onRun(encoderTask);
  steeringThread.onRun(steeringTask);
  commsThread.onRun(commsTask);
  heartbeatThread.onRun(heartbeatTask);
  motorThread.setInterval(MOTOR_TASK_PERIOD_MS);
  encoderThread.setInterval(ENCODER_TASK_PERIOD_MS);
  steeringThread.setInterval(STEER_TASK_PERIOD_MS);
  commsThread.setInterval(COMM_TASK_PERIOD_MS);
  heartbeatThread.setInterval(HEARTBEAT_TASK_PERIOD_MS);
}

void loop() {
  // Run each thread at the appropriate interval
  if (motorThread.shouldRun()) { motorThread.run(); }
  if (encoderThread.shouldRun()) { encoderThread.run(); }
  if (steeringThread.shouldRun()) { steeringThread.run(); }
  if (commsThread.shouldRun()) { commsThread.run(); }
  if (heartbeatThread.shouldRun()) { heartbeatThread.run(); }
}
