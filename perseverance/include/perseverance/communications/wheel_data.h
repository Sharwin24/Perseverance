#ifndef WHEEL_DATA_HPP
#define WHEEL_DATA_HPP

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)
struct WheelDataPacket {
  uint16_t preamble;     // 0xA55A
  uint8_t  version;      // 1
  uint8_t  msg_type;     // 0x01 = WheelData
  uint32_t seq;          // sequence counter
  float    wheelVels[6]; // [rad/s]
  float    wheelSteers[4]; // [rad]
  uint16_t crc;          // CRC-16/CCITT over bytes [preamble..wheelSteers[3]]
};
#pragma pack(pop)
static_assert(sizeof(WheelDataPacket) == 2 + 1 + 1 + 4 + (6 * 4) + (4 * 4) + 2, "unexpected packing");

static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

#endif // !WHEEL_DATA_HPP
