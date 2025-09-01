/**
 * @file bno055.cpp
 * @brief Implementation of the BNO055 sensor interface.
 *
 * This file contains the implementation of the BNO055 sensor interface,
 * including initialization, reset, and data reading functions.
 *
 * @author Dheera Venkatraman <dheera@dheera.net>
 * @note This file was modified by Sharwin Patil
 * @date 2025-03-18
 */


#include "drivers/bno055.hpp"
#include "drivers/bno055_registers.hpp"
#include "drivers/smbus_functions.h"

bool BNO055::reset() {
  _i2c_smbus_write_byte_data(this->file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // reset
  _i2c_smbus_write_byte_data(this->file, BNO055_SYS_TRIGGER_ADDR, 0x20);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  int i = 0;
  // wait for chip to come back online
  while (_i2c_smbus_read_byte_data(this->file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i++ > 500) {
      throw std::runtime_error("chip did not come back online within 5 seconds of reset");
      return false;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // normal power mode
  _i2c_smbus_write_byte_data(this->file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  _i2c_smbus_write_byte_data(this->file, BNO055_PAGE_ID_ADDR, 0);
  _i2c_smbus_write_byte_data(this->file, BNO055_SYS_TRIGGER_ADDR, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  _i2c_smbus_write_byte_data(this->file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  return true;
}

void BNO055::init(std::string i2c_device, uint8_t i2c_address) {
  this->file = open(i2c_device.c_str(), O_RDWR);

  if (ioctl(this->file, I2C_SLAVE, i2c_address) < 0) {
    throw std::runtime_error("i2c device open failed");
  }

  if (_i2c_smbus_read_byte_data(this->file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    throw std::runtime_error("incorrect chip ID");
  }

  std::cerr << "rev ids:"
    << " accel:" << _i2c_smbus_read_byte_data(this->file, BNO055_ACCEL_REV_ID_ADDR)
    << " mag:" << _i2c_smbus_read_byte_data(this->file, BNO055_MAG_REV_ID_ADDR)
    << " gyro:" << _i2c_smbus_read_byte_data(this->file, BNO055_GYRO_REV_ID_ADDR)
    << " sw:" << _i2c_smbus_read_word_data(this->file, BNO055_SW_REV_ID_LSB_ADDR)
    << " bl:" << _i2c_smbus_read_byte_data(this->file, BNO055_BL_REV_ID_ADDR) << std::endl;

  if (!this->reset()) {
    throw std::runtime_error("chip init failed");
  }
}

IMURecord BNO055::read() {
  IMURecord record;
  // can only read a length of 0x20 at a time, so do it in 2 reads
  // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
  if (_i2c_smbus_read_i2c_block_data(this->file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
    throw std::runtime_error("read error");
  }
  if (_i2c_smbus_read_i2c_block_data(this->file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
    throw std::runtime_error("read error");
  }

  return record;
}
