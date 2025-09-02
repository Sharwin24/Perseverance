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
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#include <PID_v1.h>

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

// struct WheelData {
//   // The wheel's ID [FL=0, FR=1, ML=2, MR=3, RL=4, RR=5]
//   uint8_t id;
//   // The wheel's encoder count [pulses]
//   int32_t encoder_count;
//   // The wheel's motor speed [rad/sec]
//   float motor_speed;
//   // The wheel's steering angle [radians]
//   float steering_angle;
// };

// 6 encoder positions {FL, FR, ML, MR, RL, RR} (32-bit int) [pulses]
K_MSGQ_DEFINE(encoder_msgq, sizeof(int32_t) * 6, 10, 4);
// 6 motor speeds {FL, FR, ML, MR, RL, RR} (32-bit float) [rad/sec]
K_MSGQ_DEFINE(cmd_msgq, sizeof(float) * 6, 5, 4);
// 4 servo angles {FL, FR, RL, RR} (32-bit float) [radians]
K_MSGQ_DEFINE(servo_msgq, sizeof(float) * 4, 5, 4);

// Thread stacks
K_THREAD_STACK_DEFINE(encoder_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(motor_control_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(comms_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(steering_stack, STACK_SIZE);

// Thread IDs
static struct k_thread encoder_task;
static struct k_thread motor_control_task;
static struct k_thread comms_task;
static struct k_thread steering_task;

// Global data for tasks to access
static const struct device* gpio_dev;
static const struct device* pwm_dev;
static const struct device* spi_dev;

// Global state for PID controllers
static double encoder_input[6] = {0, 0, 0, 0, 0, 0};
static double motor_output[6] = {0, 0, 0, 0, 0, 0};
static double target_velocities[6] = {0, 0, 0, 0, 0, 0};

// PID control instances for each wheel
// P, I, D constants - these will need to be tuned for your specific robot
const double Kp = 2.0;
const double Ki = 5.0;
const double Kd = 1.0;
PID pid_fl(&encoder_input[0], &motor_output[0], &target_velocities[0], Kp, Ki, Kd, DIRECT);
PID pid_fr(&encoder_input[1], &motor_output[1], &target_velocities[1], Kp, Ki, Kd, DIRECT);
PID pid_ml(&encoder_input[2], &motor_output[2], &target_velocities[2], Kp, Ki, Kd, DIRECT);
PID pid_mr(&encoder_input[3], &motor_output[3], &target_velocities[3], Kp, Ki, Kd, DIRECT);
PID pid_rl(&encoder_input[4], &motor_output[4], &target_velocities[4], Kp, Ki, Kd, DIRECT);
PID pid_rr(&encoder_input[5], &motor_output[5], &target_velocities[5], Kp, Ki, Kd, DIRECT);

// A simple structure to hold the pin information for each wheel
struct WheelPins {
  uint8_t pwm_pin;
  uint8_t dir_pin;
  uint8_t enc_a_pin;
  uint8_t enc_b_pin;
};

const struct WheelPins wheels[6] = {
    {FL_MOTOR_PWM_PIN, FL_MOTOR_DIR_PIN, FL_ENCODER_A_PIN, FL_ENCODER_B_PIN}, // FL
    {FR_MOTOR_PWM_PIN, FR_MOTOR_DIR_PIN, FR_ENCODER_A_PIN, FR_ENCODER_B_PIN}, // FR
    {ML_MOTOR_PWM_PIN, ML_MOTOR_DIR_PIN, ML_ENCODER_A_PIN, ML_ENCODER_B_PIN}, // ML
    {MR_MOTOR_PWM_PIN, MR_MOTOR_DIR_PIN, MR_ENCODER_A_PIN, MR_ENCODER_B_PIN}, // MR
    {RL_MOTOR_PWM_PIN, RL_MOTOR_DIR_PIN, RL_ENCODER_A_PIN, RL_ENCODER_B_PIN}, // RL
    {RR_MOTOR_PWM_PIN, RR_MOTOR_DIR_PIN, RR_ENCODER_A_PIN, RR_ENCODER_B_PIN}  // RR
};

// Global encoder counts, accessed by encoder and motor tasks
volatile int32_t g_encoder_counts[6] = {0};

// Forward declaration for interrupt handlers
void encoder_a_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
void encoder_b_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins);

// GPIO callbacks for encoder interrupts
static struct gpio_callback encoder_cb[6];


// Function to initialize all GPIO pins
int init_gpio_pins() {
  gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  if (!device_is_ready(gpio_dev)) {
    printk("Error: GPIO device is not ready\n");
    return -ENODEV;
  }

  // Initialize motor direction pins
  for (int i = 0; i < 6; i++) {
    gpio_pin_configure(gpio_dev, wheels[i].dir_pin, GPIO_OUTPUT);
  }

  // Initialize encoder pins with interrupts
  for (int i = 0; i < 6; i++) {
    gpio_pin_configure(gpio_dev, wheels[i].enc_a_pin, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio_dev, wheels[i].enc_a_pin, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&encoder_cb[i], encoder_a_isr, BIT(wheels[i].enc_a_pin));
    gpio_add_callback(gpio_dev, &encoder_cb[i]);

    gpio_pin_configure(gpio_dev, wheels[i].enc_b_pin, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio_dev, wheels[i].enc_b_pin, GPIO_INT_EDGE_BOTH);
    // Note: Zephyr's GPIO callback system can be tricky for multiple pins on the same callback.
    // For a more robust solution, you'd typically use a separate callback for each pin.
    // For this example, we'll assume a single callback per wheel pair (A & B) for simplicity.
    // A more advanced implementation would use dedicated hardware timers.
  }

  return 0;
}

// Encoder Task
void encoder_task_entry(void);

// Motor Control Task
void motor_control_task_entry(void);

// Communication Task
void comms_task_entry(void);

// Steering Task
void steering_task_entry(void);
/ ============ TASK IMPLEMENTATIONS ============

// Encoder Task: Reads encoders and updates counts (1kHz)
void encoder_task_entry(void) {
  if (init_gpio_pins() != 0) {
    printk("Failed to initialize GPIO pins!\n");
    return;
  }

  // In a real application, a continuous loop isn't needed here if interrupts
  // are handling the counts. The counts would be published on a timer.
  // For this example, we'll use a timer to simulate a 1kHz update
  while (1) {
    int32_t counts_to_send[6];
    for (int i = 0; i < 6; i++) {
      counts_to_send[i] = g_encoder_counts[i];
    }
    k_msgq_put(&encoder_msgq, &counts_to_send, K_NO_WAIT);
    k_sleep(K_MSEC(1)); // Run approximately every 1ms (1kHz)
  }
}

// GPIO Interrupt Handler for Encoders
void encoder_a_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
  uint32_t pin = 31 - __builtin_clz(pins); // Get the pin number from the bitmask
  int wheel_idx = -1;
  for (int i = 0; i < 6; i++) {
    if (pin == wheels[i].enc_a_pin) {
      wheel_idx = i;
      break;
    }
  }
  if (wheel_idx == -1) {
    return;
  }

  int a_state = gpio_pin_get(dev, wheels[wheel_idx].enc_a_pin);
  int b_state = gpio_pin_get(dev, wheels[wheel_idx].enc_b_pin);

  if (a_state != 0) {
    if (b_state != 0) {
      g_encoder_counts[wheel_idx]++;
    }
    else {
      g_encoder_counts[wheel_idx]--;
    }
  }
  else {
    if (b_state != 0) {
      g_encoder_counts[wheel_idx]--;
    }
    else {
      g_encoder_counts[wheel_idx]++;
    }
  }
}


// Motor Control Task: PID control loop (200Hz)
void motor_control_task_entry(void) {
  // Initialize PWM device
  pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
  if (!device_is_ready(pwm_dev)) {
    printk("Error: PWM device is not ready\n");
    return;
  }

  // Set up PID controllers
  pid_fl.SetMode(AUTOMATIC);
  pid_fr.SetMode(AUTOMATIC);
  pid_ml.SetMode(AUTOMATIC);
  pid_mr.SetMode(AUTOMATIC);
  pid_rl.SetMode(AUTOMATIC);
  pid_rr.SetMode(AUTOMATIC);

  // Set PID output limits to match PWM range (e.g., 0-255 or 0-100)
  pid_fl.SetOutputLimits(-255, 255);
  // Repeat for all 6 PIDs...

  while (1) {
    // Read encoder counts from queue
    int32_t current_counts[6];
    k_msgq_get(&encoder_msgq, &current_counts, K_FOREVER);

    // Read command velocities from queue
    // If a new command is not available, use the last one
    k_msgq_get(&cmd_msgq, &target_velocities, K_NO_WAIT);

    // Update PID inputs
    for (int i = 0; i < 6; i++) {
      encoder_input[i] = (double)current_counts[i]; // Replace with actual velocity calculation
    }

    // Compute new motor outputs
    pid_fl.Compute();
    pid_fr.Compute();
    pid_ml.Compute();
    pid_mr.Compute();
    pid_rl.Compute();
    pid_rr.Compute();

    // Update motor PWM and direction based on PID output
    for (int i = 0; i < 6; i++) {
      if (motor_output[i] >= 0) {
        gpio_pin_set(gpio_dev, wheels[i].dir_pin, 1);
      }
      else {
        gpio_pin_set(gpio_dev, wheels[i].dir_pin, 0);
      }
      // Zephyr's PWM API uses pulse width, so we need to map the PID output
      uint32_t pulse_width = (uint32_t)fabs(motor_output[i]);
      // You will need to determine the correct PWM period and channel
      // based on the specific Teensy 4.1 hardware and your PWM configuration
      // For example: pwm_pin_set_cycles(pwm_dev, wheels[i].pwm_pin, 255, pulse_width, 0);
    }

    k_msleep(MOTOR_TASK_PERIOD);
  }
}

// Communication Task: Handle SPI communication (100Hz)
void comms_task_entry(void) {
  // Initialize SPI
  spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
  if (!device_is_ready(spi_dev)) {
    printk("Error: SPI device is not ready\n");
    return;
  }

  // SPI configuration as a slave
  struct spi_config spi_cfg = {
      .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
      .frequency = 1000000, // Example frequency, tune as needed
      .slave = 1, // Set Teensy as a slave
  };

  while (1) {
    // --- Receiving Data from RPI5 ---
    // A simple protocol for SPI communication would be needed here.
    // e.g., an SPI master sends a header byte, then sends the data.
    // We'll simulate this by just receiving a mock command message.
    float incoming_motor_cmds[6];
    float incoming_servo_cmds[4];

    // This is a placeholder for actual SPI receive logic.
    // In reality, this would be a more complex state machine
    // handling SPI master requests.
    // spi_transceive(spi_dev, &rx_buf, &tx_buf)

    // For now, we'll just put mock data into the queues
    k_msgq_put(&cmd_msgq, &incoming_motor_cmds, K_NO_WAIT);
    k_msgq_put(&servo_msgq, &incoming_servo_cmds, K_NO_WAIT);

    // --- Sending Data to RPI5 ---
    int32_t encoder_data[6];
    k_msgq_get(&encoder_msgq, &encoder_data, K_NO_WAIT);

    struct spi_buf tx_buf = {.buf = encoder_data, .len = sizeof(encoder_data)};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

    spi_transceive(spi_dev, &spi_cfg, &tx_set, NULL);

    k_msleep(COMM_TASK_PERIOD);
  }
}

// Steering Task: Control servos (100Hz)
void steering_task_entry(void) {
  // Initialize PWM device for servos
  pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
  if (!device_is_ready(pwm_dev)) {
    printk("Error: PWM device is not ready\n");
    return;
  }

  const uint32_t servo_pins[4] = {FL_STEERING_SERVO_PIN, FR_STEERING_SERVO_PIN, RL_STEERING_SERVO_PIN, RR_STEERING_SERVO_PIN};

  while (1) {
    float target_angles[4];
    k_msgq_get(&servo_msgq, &target_angles, K_FOREVER);

    for (int i = 0; i < 4; i++) {
      // Convert angle (radians) to PWM pulse width (microseconds)
      // A standard servo range is from 500us to 2500us
      // 0 radians -> 1500us (center)
      // Max angle -> 2500us
      // Min angle -> 500us
      uint32_t pulse_width = 1500 + (uint32_t)(target_angles[i] * 1000); // Placeholder conversion

      // Set the PWM for the servo
      pwm_set_cycles(pwm_dev, servo_pins[i], SERVO_PWM_PERIOD, pulse_width, 0);
    }

    k_msleep(STEER_TASK_PERIOD);
  }
}

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
