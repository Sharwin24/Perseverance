/**
 * @file bno055.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Header file containing class definition for BNO055 that interfaces
 * with the BNO055 IMU sensor over I2C.
 * @date 2025-03-18
 *
 */

#ifndef _BNO055_HPP_
#define _BNO055_HPP_

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <linux/i2c-dev.h>
#include "smbus_functions.h"

#include "bno055_registers.hpp"

 /**
  * @brief Struct for storing the BNO055 sensor data
  *
  * @note The order of this struct is designed to match the I2C registers
  * so all data can be read in one fell swoop
  *
  */
typedef struct {
  int16_t raw_linear_acceleration_x; /** @brief Raw Linear Accel X */
  int16_t raw_linear_acceleration_y; /** @brief Raw Linear Accel Y */
  int16_t raw_linear_acceleration_z; /** @brief Raw Linear Accel Z */
  int16_t raw_magnetic_field_x; /** @brief Raw Magnetic Field X */
  int16_t raw_magnetic_field_y; /** @brief Raw Magnetic Field Y */
  int16_t raw_magnetic_field_z; /** @brief Raw Magnetic Field Z */
  int16_t raw_angular_velocity_x; /** @brief Raw Angular Velocity X */
  int16_t raw_angular_velocity_y; /** @brief Raw Angular Velocity Y */
  int16_t raw_angular_velocity_z; /** @brief Raw Angular Velocity Z */
  int16_t fused_heading; /** @brief Heading obtained from sensor fusion */
  int16_t fused_roll; /** @brief Roll obtained from sensor fusion */
  int16_t fused_pitch; /** @brief Pitch obtained from sensor fusion */
  int16_t fused_orientation_w; /** @brief W component of the quaternion obtained from sensor fusion */
  int16_t fused_orientation_x; /** @brief X component of the quaternion obtained from sensor fusion */
  int16_t fused_orientation_y; /** @brief Y component of the quaternion obtained from sensor fusion */
  int16_t fused_orientation_z; /** @brief Z component of the quaternion obtained from sensor fusion */
  int16_t fused_linear_acceleration_x; /** @brief Linear Accel X obtained from sensor fusion */
  int16_t fused_linear_acceleration_y; /** @brief Linear Accel Y obtained from sensor fusion */
  int16_t fused_linear_acceleration_z; /** @brief Linear Accel Z obtained from sensor fusion */
  int16_t gravity_vector_x; /** @brief Gravity Vector X */
  int16_t gravity_vector_y; /** @brief Gravity Vector Y */
  int16_t gravity_vector_z; /** @brief Gravity Vector Z */
  int8_t temperature; /** @brief Temperature [C] */
  uint8_t calibration_status; /** @brief Calibration status */
  uint8_t self_test_result; /** @brief Self test result */
  uint8_t interrupt_status; /** @brief Interrupt status */
  uint8_t system_clock_status; /** @brief System clock status */
  uint8_t system_status; /** @brief System status */
  uint8_t system_error_code; /** @brief System error code */
} IMURecord;

/**
 * @class BNO055
 * @brief A class to interface with the BNO055 IMU sensor over I2C
 *
 */
class BNO055 {
public:
  /**
   * @brief Construct a new BNO055 object
   *
   */
  BNO055() = default;

  /**
   * @brief Destroy the BNO055 object
   *
   */
  ~BNO055() = default;

  /**
   * @brief Initialize the I2C device and establish communication
   *
   * @param i2c_dev the device name for the hardware I2C on the system
   * @param i2c_addr the I2C address of the BNO055 sensor
   */
  void init(std::string i2c_dev, uint8_t i2c_addr);

  /**
   * @brief Attempts to reset the sensor
   *
   * @return true if the reset was successful
   * @return false if the reset failed for any reason
   */
  bool reset();

  /**
   * @brief Read all of the sensor data and pack it into one struct to be returned
   *
   * @return IMURecord the struct containing all of the sensor data
   */
  IMURecord read();

private:
  /**
  * @brief The file descriptor for I2C device
  *
  */
  int file;
};

#endif // _BNO055_HPP_