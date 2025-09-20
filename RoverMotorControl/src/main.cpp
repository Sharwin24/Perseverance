/**
 * @file main.cpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Rover's motor firmware using an Adafruit Feather M4 Express
 * @date 2024-09-019
 * @version 3.0
 *
 * This board needs to interact with the following hardware:
 * 1. 2 MotorShield Featherwings to control 6 DC motors (3 per shield) over I2C
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
#include <Adafruit_MotorShield.h>
#include <Adafruit_BusIO_Register.h>

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "rover_pins.h"

#define SERIAL_BAUD 115200 // Baud rate for Serial communication
#define PWM_FREQ 20000 // 20 kHz PWM frequency for motor control

 // --- Task Frequencies ---
#define MOTOR_TASK_FREQ 100 // [Hz]
#define MOTOR_TASK_PERIOD_MS (1000 / MOTOR_TASK_FREQ)
#define ENCODER_TASK_FREQ 1000 // [Hz]
#define ENCODER_TASK_PERIOD_MS (1000 / ENCODER_TASK_FREQ)
#define COMM_TASK_FREQ 100 // [Hz]
#define COMM_TASK_PERIOD_MS (1000 / COMM_TASK_FREQ)
#define STEER_TASK_FREQ 100 // [Hz]
#define STEER_TASK_PERIOD_MS (1000 / STEER_TASK_FREQ)
#define HEARTBEAT_TASK_FREQ 1 // [Hz]
#define HEARTBEAT_TASK_PERIOD_MS (1000 / HEARTBEAT_TASK_FREQ)

// Task Stack Sizes
#define TASK_STACK_SIZE 2048

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

// --- Shared Data and Mutexes ---
// Use volatile for variables shared between threads
volatile float motorSpeedCommands[6] = {0, 0, 0, 0, 0, 0}; // {FL, FR, ML, MR, RL, RR} [rad/s]
volatile float servoAngleCommands[4] = {0, 0, 0, 0}; // {FL, FR, RL, RR} [radians]
volatile long encoderCounts[6] = {0, 0, 0, 0, 0, 0}; // {FL, FR, ML, MR, RL, RR} [ticks]

SemaphoreHandle_t mutexSPI; // Mutex to protect shared data access

// Global timing variables for tasks
uint32_t prevMotorUpdateUs = 0;
long prevEncoderCounts[6] = {0, 0, 0, 0, 0, 0};

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

// ---------- Task Functions ----------

/**
 * @brief Encoder task running at 1kHz
 *
 * @note Stores latest encoder values from hardware encoders (ISR driven)
 * into shared encoderCounts array which will be communicated to RPI5
 * and used for PID feedback
 *
 */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    for (uint8_t i = 0; i < 6; ++i) {
      encoderCounts[i] = encoders[i].read();
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ENCODER_TASK_PERIOD_MS));
  }
}

/**
 * @brief Motor control task running at 200Hz
 *
 * @note Implements a PID control loop for each drive motor
 * using the encoder feedback to calculate motor velocities
 *
 */
void MotorTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    unsigned long now = millis();
    double dt = (now - prevMotorUpdate) / 1000.0;
    prevMotorUpdate = now;

    // Copy shared data to local variables to minimize critical section time
    double local_motor_cmd[6];
    if (xSemaphoreTake(mutexSPI, portMAX_DELAY) == pdTRUE) {
      for (uint8_t i = 0; i < 6; ++i) {
        local_motor_cmd[i] = motorSpeedCommands[i];
      }
      xSemaphoreGive(mutexSPI);
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
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
  }
}


/**
 * @brief Steering task running at 100Hz
 *
 * @note Accesses shared servoAngleCommands array to set servo positions
 *
 */
void SteeringTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    // Copy shared data to local variables
    float local_servo_angles[4];
    if (xSemaphoreTake(mutexSPI, portMAX_DELAY) == pdTRUE) {
      for (uint8_t i = 0; i < 4; i++) {
        local_servo_angles[i] = servoAngleCommands[i];
      }
      xSemaphoreGive(mutexSPI);
    }

    for (uint8_t i = 0; i < 4; i++) {
      float degrees = degrees(local_servo_angles[i]);
      servos[i].write(map(degrees, -90, 90, 0, 180));
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STEER_TASK_PERIOD_MS));
  }
}

/**
 * @brief Communication task running at 100Hz
 *
 * @note Handles SPI communication with RPI5 and accesses shared command arrays
 *
 */
void CommsTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    uint8_t txBuffer[24];
    uint8_t rxBuffer[40];

    // SPI slave receives data, then sends data
    SPI.transfer(0, rxBuffer, 40, 0);

    float received_motor_speeds[6];
    for (int i = 0; i < 6; ++i) {
      union { uint32_t u; float f; } u32f;
      memcpy(&u32f.u, &rxBuffer[i * 4], 4);
      received_motor_speeds[i] = u32f.f;
    }

    float received_servo_angles[4];
    for (int i = 0; i < 4; ++i) {
      union { uint32_t u; float f; } u32f;
      memcpy(&u32f.u, &rxBuffer[24 + i * 4], 4);
      received_servo_angles[i] = u32f.f;
    }

    if (xSemaphoreTake(mutexSPI, portMAX_DELAY) == pdTRUE) {
      memcpy((void*)motorSpeedCommands, received_motor_speeds, sizeof(received_motor_speeds));
      memcpy((void*)servoAngleCommands, received_servo_angles, sizeof(received_servo_angles));
      xSemaphoreGive(mutexSPI);
    }

    if (xSemaphoreTake(mutexSPI, portMAX_DELAY) == pdTRUE) {
      for (uint8_t i = 0; i < 6; ++i) {
        uint32_t count = (uint32_t)encoderCounts[i];
        memcpy(&txBuffer[i * 4], &count, 4);
      }
      xSemaphoreGive(mutexSPI);
    }

    SPI.transfer(txBuffer, NULL, 24, 0);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(COMM_TASK_PERIOD_MS));
  }
}

/**
 * @brief Heartbeat task running at 1Hz
 *
 * @note Indicates system is running
 *
 */
void HeartbeatTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    digitalWrite(DEBUG_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(DEBUG_LED, LOW);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HEARTBEAT_TASK_PERIOD_MS));
  }
}


void setup() {
  // Initialize Serial port
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SPI_CS_PIN, INPUT_PULLUP);

  // Attach servo motors
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(servo_pins[i], OUTPUT);
    servos[i].attach(servo_pins[i]);
  }

  // Set up PID controllers
  for (uint8_t i = 0; i < 6; ++i) {
    pid_controllers[i].SetMode(AUTOMATIC);
    pid_controllers[i].SetOutputLimits(-4095, 4095); // Use 12-bit PWM
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

  // Configure SPI slave mode
  SPI.setSCK(SPI_SCK_PIN);
  SPI.setMISO(SPI_MISO_PIN);
  SPI.setMOSI(SPI_MOSI_PIN);
  SPI.begin_slave(SPI_CS_PIN);

  mutexSPI = xSemaphoreCreateMutex();

  xTaskCreate(EncoderTask, "Encoder", TASK_STACK_SIZE, NULL, 4, NULL);
  xTaskCreate(MotorTask, "Motor", TASK_STACK_SIZE, NULL, 3, NULL);
  xTaskCreate(SteeringTask, "Steering", TASK_STACK_SIZE, NULL, 2, NULL);
  xTaskCreate(CommsTask, "Comms", TASK_STACK_SIZE, NULL, 2, NULL);
  xTaskCreate(HeartbeatTask, "Heartbeat", TASK_STACK_SIZE, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Not used when using FreeRTOS
}
