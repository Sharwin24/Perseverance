/**
 * @file main.cpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Main control code for the Perseverance Rover's motor and servo control using Teensy 4.1
 * @version 1.0
 *
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
#include <TeensyThreads.h>
#include <SPISlave_T4.h>

#include "rover_pins.h"

#define SERIAL_BAUD 115200 // Baud rate for Serial communication
#define PWM_FREQ 20000 // 20 kHz PWM frequency for motor control

 // --- Task Frequencies ---
#define MOTOR_TASK_FREQ 200 // [Hz]
#define MOTOR_TASK_PERIOD_MS (1000 / MOTOR_TASK_FREQ)
#define COMM_TASK_FREQ 100 // [Hz]
#define COMM_TASK_PERIOD_MS (1000 / COMM_TASK_FREQ)
#define STEER_TASK_FREQ 100 // [Hz]
#define STEER_TASK_PERIOD_MS (1000 / STEER_TASK_FREQ)

// --- Shared Data and Mutexes ---
// Use volatile for variables shared between threads
volatile float motor_speed_commands[6] = {0, 0, 0, 0, 0, 0};
volatile float servo_angle_commands[4] = {0, 0, 0, 0};
volatile long encoder_counts[6] = {0, 0, 0, 0, 0, 0};
Threads::Mutex comms_mutex;

// A simple structure to hold the pin information for each wheel
struct WheelPins {
  uint8_t pwm_pin;
  uint8_t dir_pin;
};

const WheelPins wheels[6] = {
    {FL_MOTOR_PWM_PIN, FL_MOTOR_DIR_PIN}, // FL
    {FR_MOTOR_PWM_PIN, FR_MOTOR_DIR_PIN}, // FR
    {ML_MOTOR_PWM_PIN, ML_MOTOR_DIR_PIN}, // ML
    {MR_MOTOR_PWM_PIN, MR_MOTOR_DIR_PIN}, // MR
    {RL_MOTOR_PWM_PIN, RL_MOTOR_DIR_PIN}, // RL
    {RR_MOTOR_PWM_PIN, RR_MOTOR_DIR_PIN}  // RR
};

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


// 64-byte SPI frame, same both directions.
// Master (Pi) fills the command fields; slave (Teensy) fills the encoder fields.
#pragma pack(push, 1)
struct SpiFrame {
  uint16_t magic;          // 0xA55A
  uint8_t  type;           // 0x01 = normal
  uint8_t  seq;            // rolling sequence
  int16_t  motor_cmd[6];   // ticks/s or rpm * SCALE_CMD
  int16_t  servo_cmd[4];   // radians * SCALE_SERVO (e.g., 1000 = mrad)
  int32_t  enc_counts[6];  // absolute encoder counts (Teensy->Pi)
  uint32_t crc32;          // CRC-32 of bytes [0..(offset of crc32 - 1)]
  uint8_t  pad[64 - (2 + 1 + 1 + 6 * 2 + 4 * 2 + 6 * 4 + 4)];
};
#pragma pack(pop)

static constexpr float  SCALE_CMD = 100.0f;   // 1 unit = 0.01 of your native unit
static constexpr float  SCALE_SERVO = 1000.0f;  // 1 unit = 0.001 rad
static constexpr uint16_t MAGIC = 0xA55A;


SPISlave_T4<> spiSlave;    // default LPSPI bus (pins: CS=10, SCK=13, MISO=12, MOSI=11)

// Shared frame buffer (double-buffer to avoid tearing)
volatile SpiFrame rx_frame{};
volatile SpiFrame tx_frame{};
volatile uint8_t last_good_seq = 0;

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  // simple, fast CRC-32 (poly 0xEDB88320); you can drop in any standard impl
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  }
  return ~crc;
}

void spi_rx_cb() {
  // Get the received data and its length from the SPISlave_T4 API
  uint16_t len = spiSlave.available();
  if (len != sizeof(SpiFrame)) return;  // ignore partial frames

  uint8_t buf[sizeof(SpiFrame)];
  spiSlave.read(buf, sizeof(SpiFrame));
  memcpy((void*)&rx_frame, buf, sizeof(SpiFrame));

  // Validate
  const auto* f = (const SpiFrame*)&rx_frame;
  if (f->magic != MAGIC) return;
  uint32_t calc = crc32_update(0, (const uint8_t*)f, offsetof(SpiFrame, crc32));
  if (calc != f->crc32) return;

  last_good_seq = f->seq;

  // Apply commands (protect with your mutex)
  Threads::Scope lock(comms_mutex);
  for (int i = 0; i < 6; ++i) {
    motor_speed_commands[i] = (float)f->motor_cmd[i] / SCALE_CMD;
  }
  for (int i = 0; i < 4; ++i) {
    servo_angle_commands[i] = (float)f->servo_cmd[i] / SCALE_SERVO; // radians
  }

  // Prepare the *next* TX frame with latest encoders
  SpiFrame t{};
  t.magic = MAGIC;
  t.type = 0x81; // reply
  t.seq = last_good_seq;
  for (int i = 0; i < 6; ++i) t.enc_counts[i] = encoder_counts[i];
  t.crc32 = crc32_update(0, (const uint8_t*)&t, offsetof(SpiFrame, crc32));
  memcpy((void*)&tx_frame, &t, sizeof(SpiFrame));
}

void spi_tx_cb(uint8_t* data, uint16_t len) {
  // Library asks what to shift out: provide our tx_frame
  if (len == sizeof(SpiFrame)) {
    memcpy(data, (const void*)&tx_frame, sizeof(SpiFrame));
  }
  else {
    // If master does shorter transfer, just fill zeros
    memset(data, 0, len);
  }
}

void startSpiSlave() {
  // Default pins: CS=10, SCK=13, MISO=12, MOSI=11. Change with spiSlave.setCS(), etc.
  spiSlave.begin();
  spiSlave.onReceive(spi_rx_cb);
  spiSlave.onTransmit(spi_tx_cb);

  // Prime the first reply so the master doesn’t read garbage on the first cycle
  SpiFrame t{};
  t.magic = MAGIC; t.type = 0x81; t.seq = 0;
  for (int i = 0; i < 6; ++i) t.enc_counts[i] = 0;
  t.crc32 = crc32_update(0, (const uint8_t*)&t, offsetof(SpiFrame, crc32));
  memcpy((void*)&tx_frame, &t, sizeof(SpiFrame));
}


// --- Task Functions ---

// Encoder Task: Reads and updates encoder counts at 1kHz
void encoderTask() {
  elapsedMicros us = 0;
  for (;;) {
    if (us >= 1000) {      // 1000 us = 1 kHz
      us -= 1000;
      for (uint8_t i = 0; i < 6; ++i) encoder_counts[i] = encoders[i].read();
    }
    threads.yield();
  }
}

// Motor Control Task: PID control loop for drive motors (200Hz)
void motorTask() {
  unsigned long last = millis();
  static long prev_counts[6] = {};
  while (true) {
    unsigned long now = millis();
    if (now - last >= MOTOR_TASK_PERIOD_MS) {
      double dt = (now - last) / 1000.0; last = now;

      double local_motor_cmd[6];
      {
        Threads::Scope lock(comms_mutex);
        for (int i = 0; i < 6; ++i) local_motor_cmd[i] = motor_speed_commands[i];
      }

      for (int i = 0; i < 6; ++i) {
        long c = encoder_counts[i];
        double vel = (double)(c - prev_counts[i]) / dt; // ticks/s
        prev_counts[i] = c;

        pid_inputs[i] = vel;
        pid_targets[i] = local_motor_cmd[i];
        pid_controllers[i].Compute();

        int pwm = constrain((int)fabs(pid_outputs[i]), 0, 255);
        digitalWrite(wheels[i].dir_pin, (pid_outputs[i] >= 0) ? HIGH : LOW);
        analogWrite(wheels[i].pwm_pin, pwm);
      }
    }
    threads.yield();
  }
}


// Steering Task: Control steering servos (100Hz)
void steeringTask() {
  unsigned long lastTime = millis();
  while (true) {
    unsigned long now = millis();
    if (now - lastTime >= STEER_TASK_PERIOD_MS) {
      lastTime = now;
      for (int i = 0; i < 4; i++) {
        // Convert radians to degrees for Servo library
        float degrees = servo_angle_commands[i] * 57.2958;
        servos[i].write(map(degrees, -90, 90, 0, 180));
      }
    }
    threads.yield();
  }
}

// Communication Task: Handle SPI communication with RPi5 (100Hz)
void commsTask() {
  SPI.setCS(SPI_CS_PIN);
  SPI.setMISO(SPI_MISO_PIN);
  SPI.setMOSI(SPI_MOSI_PIN); // Default, but good practice to be explicit
  SPI.setSCK(SPI_SCK_PIN); // Default, but good practice to be explicit
  SPI.begin(); // Initialize SPI as master (default mode)

  // NOTE: Teensy SPI library does not support slave mode via SPI.begin_slave().
  // If you need SPI slave functionality, consider using a dedicated library such as "SPI Slave" for Teensy.
  // Your custom SPI protocol to handle bi-directional data
  // This is a placeholder for your custom receive/send logic

  while (true) {
    // Check for data from the RPi Master
    // ...
    float received_motor_speeds[6] = {0}; // Placeholder for received data
    float received_servo_angles[4] = {0}; // Placeholder for received data

    {
      Threads::Scope lock = Threads::Scope(comms_mutex); // Lock shared data
      for (int i = 0; i < 6; i++) {
        motor_speed_commands[i] = received_motor_speeds[i];
      }
      for (int i = 0; i < 4; i++) {
        servo_angle_commands[i] = received_servo_angles[i];
      }
    }

    // Send data to the RPi Master
    // ...

    threads.sleep(COMM_TASK_PERIOD_MS);
  }
}

// --- Arduino Sketch Functions ---
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Initialize motor direction pins
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(wheels[i].dir_pin, OUTPUT);
    digitalWrite(wheels[i].dir_pin, LOW);
    pinMode(wheels[i].pwm_pin, OUTPUT);
    analogWriteFrequency(wheels[i].pwm_pin, PWM_FREQ);
    analogWriteResolution(8); // 8-bit resolution (0-255) for simplicity
  }

  // Attach servo motors
  for (uint8_t i = 0; i < 4; i++) {
    servos[i].attach(servo_pins[i]);
  }

  // Set up PID controllers
  for (int i = 0; i < 6; ++i) {
    pid_controllers[i].SetMode(AUTOMATIC);
    pid_controllers[i].SetOutputLimits(-255, 255);
    pid_controllers[i].SetSampleTime(MOTOR_TASK_PERIOD_MS); // = 5 ms for 200 Hz
  }

  // Initialize SPI in slave mode
  startSpiSlave();

  // Create and start the tasks with priorities
  threads.addThread(encoderTask, 0, 1024); // Highest priority
  threads.addThread(motorTask, 1, 1024);
  threads.addThread(steeringTask, 2, 1024);
  threads.addThread(commsTask, 3, 1024);

  // Keep the main loop simple
  pinMode(DEBUG_LED, OUTPUT);
}

void loop() {
  // A simple heartbeat to show the board is alive
  digitalWrite(DEBUG_LED, HIGH);
  delay(100);
  digitalWrite(DEBUG_LED, LOW);
  delay(900);
}
