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
 * 1. Encoder Task: Reading Encoders and saving encoder count (1kHz)
 * 2. Motor Task: PID Control loop for drive motors and updating PWM signals (200Hz)
 * 3. Communication Task: Handle SPI communication with RPI5 (100Hz)
 * 4. Steering Task: Control steering servos based on RPI5 commands (100Hz)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/spi.h>

#include "rover_pins.h"

 // Stack size for tasks [bytes]
#define STACK_SIZE 1024
// High Priority for Encoder Task
#define ENC_PRIORITY 7
// Medium Priority for Motor Control Task
#define MOTOR_PRIORITY 6
// Low-Medium Priority for Communications Task
#define COMM_PRIORITY 5
// Low-Medium Priority for Steering Task
#define STEER_PRIORITY 5
// Motor Control Task Frequency [Hz]
#define MOTOR_TASK_FREQ 200
// Motor Control Task Period [ms]
#define MOTOR_TASK_PERIOD (1000 / MOTOR_TASK_FREQ)
// Communication Task Frequency [Hz]
#define COMM_TASK_FREQ 100
// Communication Task Period [ms]
#define COMM_TASK_PERIOD (1000 / COMM_TASK_FREQ)
// Steering Task Frequency [Hz]
#define STEER_TASK_FREQ 100
// Steering Task Period [ms]
#define STEER_TASK_PERIOD (1000 / STEER_TASK_FREQ)

struct WheelData {
  // The wheel's ID [FL=0, FR=1, ML=2, MR=3, RL=4, RR=5]
  uint8_t id;
  // The wheel's encoder count [pulses]
  int32_t encoder_count;
  // The wheel's motor speed [rad/sec]
  float motor_speed;
  // The wheel's steering angle [radians]
  float steering_angle;
};

// 6 encoder positions {FL, FR, ML, MR, RL, RR} (32-bit int) [pulses]
K_MSGQ_DEFINE(encoder_msgq, sizeof(int32_t) * 6, 10, 4);
// 6 motor speeds {FL, FR, ML, MR, RL, RR} (32-bit float) [rad/sec]
K_MSGQ_DEFINE(cmd_msgq, sizeof(float) * 6, 5, 4);
// 4 servo angles {FL, FR, RL, RR} (32-bit float) [radians]
K_MSGQ_DEFINE(servo_msgq, sizeof(float) * 4, 5, 4);

// Encoder Task
void encoder_task_entry(void);

// Motor Control Task
void motor_control_task_entry(void);

// Communication Task
void comms_task_entry(void);

// Steering Task
void steering_task_entry(void);

void main(void) {
  // Create and start the tasks with their defined priorities
  k_thread_create(
    &encoder_task,
    encoder_stack,
    STACK_SIZE,
    encoder_task_entry,
    NULL, NULL, NULL,
    ENC_PRIORITY,
    0,
    K_NO_WAIT);

  k_thread_create(
    &motor_control_task,
    motor_control_stack,
    STACK_SIZE,
    motor_control_task_entry,
    NULL, NULL, NULL,
    MOTOR_PRIORITY,
    0,
    K_NO_WAIT);

  k_thread_create(
    &steering_task,
    steering_stack,
    STACK_SIZE,
    steering_task_entry,
    NULL, NULL, NULL,
    STEER_PRIORITY,
    0,
    K_NO_WAIT);

  k_thread_create(
    &comms_task,
    comms_stack,
    STACK_SIZE,
    comms_task_entry,
    NULL, NULL, NULL,
    COMM_PRIORITY,
    0,
    K_NO_WAIT);
}

// ============ TASK IMPLEMENTATIONS ============

void encoder_task_entry(void) {
  // Initialization code for GPIO interrupts
  // ...

  while (1) {
    // Read encoder counts for all 6 motors
    // ...

    // Put the counts into a message queue
    k_msgq_put(&encoder_msgq, &encoder_counts, K_NO_WAIT);
  }
}

void motor_control_task_entry(void) {
  // Initialization code for PWM drivers
  // ...

  while (1) {
    // Wait for and get the latest encoder counts
    k_msgq_get(&encoder_msgq, &current_counts, K_FOREVER);

    // Wait for and get the latest commands
    // If no new command, use the last one
    k_msgq_get(&cmd_msgq, &target_velocities, K_NO_WAIT);

    // Run the PID control for each motor
    // ...

    // Update motor PWM outputs
    // ...

    k_msleep(MOTOR_TASK_PERIOD); // Run approximately every 5 ms (200 Hz)
  }
}

void comms_task_entry(void) {
  // Initialization code for SPI
  // ...

  while (1) {
    // Communicate with Raspberry Pi 5
    // ...

    // Populate command and servo message queues read from RPI5
    k_msgq_put(&cmd_msgq, &motor_commands, K_NO_WAIT);
    k_msgq_put(&servo_msgq, &servo_commands, K_NO_WAIT);

    // Send encoder counts to RPI5 from queue
    k_msgq_get(&encoder_msgq, &encoder_counts, K_NO_WAIT);
    k_spi_transceive(&spi_dev, &encoder_counts, sizeof(encoder_counts), NULL, 0);

    k_msleep(COMM_TASK_PERIOD); // Run approximately every 10 ms (100 Hz)
  }
}

void steering_task_entry(void) {
  // Initialization code for steering servos
  // ...

  while (1) {
    // Wait for and get the latest servo commands
    k_msgq_get(&servo_msgq, &target_angles, K_FOREVER);

    // Update servo PWM outputs
    // ...

    k_msleep(STEER_TASK_PERIOD); // Run approximately every 10 ms (100 Hz)
  }
}