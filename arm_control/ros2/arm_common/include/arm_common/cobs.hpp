// Consistent Overhead Byte Stuffing (COBS). Removes every 0x00 from a buffer
// so 0x00 can be used as an unambiguous frame delimiter on the UART. Worst-case
// overhead is 1 byte per 254 bytes of input, plus 1.
#pragma once

#include <cstddef>
#include <cstdint>

namespace arm {

// Maximum encoded size for `n` input bytes (without the trailing delimiter).
constexpr std::size_t cobsMaxEncodedSize(std::size_t n) { return n + n / 254 + 1; }

// Encodes `len` bytes from `in` into `out`. Returns the number of bytes written,
// or 0 if `out_cap` is too small. The output never contains 0x00.
inline std::size_t cobsEncode(const uint8_t* in, std::size_t len, uint8_t* out, std::size_t out_cap) {
  if (out_cap < cobsMaxEncodedSize(len)) return 0;
  std::size_t read = 0;
  std::size_t write = 1;   // position of next data byte
  std::size_t code_pos = 0;  // position of the pending length code
  uint8_t code = 1;
  while (read < len) {
    const uint8_t b = in[read++];
    if (b == 0) {
      out[code_pos] = code;
      code_pos = write++;
      code = 1;
    } else {
      out[write++] = b;
      if (++code == 0xFF) {
        out[code_pos] = code;
        code_pos = write++;
        code = 1;
      }
    }
  }
  out[code_pos] = code;
  return write;
}

// Decodes `len` COBS bytes (no delimiter) from `in` into `out`. Returns the
// decoded length, or 0 on a malformed input or insufficient `out_cap`.
// A zero-length decoded payload is also reported as 0; callers treat it as invalid.
inline std::size_t cobsDecode(const uint8_t* in, std::size_t len, uint8_t* out, std::size_t out_cap) {
  std::size_t read = 0;
  std::size_t write = 0;
  while (read < len) {
    const uint8_t code = in[read++];
    if (code == 0) return 0;  // delimiter inside a frame: corrupt
    for (uint8_t i = 1; i < code; ++i) {
      if (read >= len) return 0;  // truncated run
      if (write >= out_cap) return 0;
      const uint8_t b = in[read++];
      if (b == 0) return 0;
      out[write++] = b;
    }
    if (code != 0xFF && read < len) {
      if (write >= out_cap) return 0;
      out[write++] = 0;
    }
  }
  return write;
}

}  // namespace arm
