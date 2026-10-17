// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
// Check value for "123456789" is 0x29B1. Bitwise implementation: no table,
// so it costs nothing in flash on the MCU and is fast enough at our rates
// (~100 bytes per frame, 200 frames/s).
#pragma once

#include <cstddef>
#include <cstdint>

namespace arm {

inline uint16_t crc16Update(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (int i = 0; i < 8; ++i) {
    crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

inline uint16_t crc16(const uint8_t* data, std::size_t len, uint16_t crc = 0xFFFFu) {
  for (std::size_t i = 0; i < len; ++i) crc = crc16Update(crc, data[i]);
  return crc;
}

}  // namespace arm
