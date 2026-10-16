// Frame = COBS( payload || CRC16(payload) little-endian ) || 0x00
//
// encodeFrame() builds a frame from any protocol struct. FrameParser consumes
// a byte stream on the receiving side and yields validated payloads. It is
// allocation-free and has no callbacks, so it runs identically on the host
// and inside the firmware's comms task.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "arm_common/cobs.hpp"
#include "arm_common/crc16.hpp"
#include "arm_common/protocol.hpp"

namespace arm {

constexpr uint8_t kFrameDelimiter = 0x00;
constexpr std::size_t kCrcSize = 2;

// Buffer size that can hold the largest frame this protocol produces.
constexpr std::size_t kMaxFrameSize = cobsMaxEncodedSize(kMaxPayloadSize + kCrcSize) + 1;

// Writes a complete frame (including the trailing delimiter) for `len` bytes
// of `payload` into `out`. Returns bytes written, 0 if `out_cap` is too small.
inline std::size_t encodeFrame(const void* payload, std::size_t len, uint8_t* out, std::size_t out_cap) {
  if (len == 0 || len > kMaxPayloadSize) return 0;
  uint8_t raw[kMaxPayloadSize + kCrcSize];
  std::memcpy(raw, payload, len);
  const uint16_t crc = crc16(raw, len);
  raw[len] = static_cast<uint8_t>(crc & 0xFFu);
  raw[len + 1] = static_cast<uint8_t>(crc >> 8);

  if (out_cap < cobsMaxEncodedSize(len + kCrcSize) + 1) return 0;
  const std::size_t n = cobsEncode(raw, len + kCrcSize, out, out_cap - 1);
  if (n == 0) return 0;
  out[n] = kFrameDelimiter;
  return n + 1;
}

template <typename T>
inline std::size_t encodeFrame(const T& msg, uint8_t* out, std::size_t out_cap) {
  return encodeFrame(&msg, sizeof(T), out, out_cap);
}

// Decodes one delimiter-free frame body. Returns payload length (>= 1) on a
// valid CRC, 0 otherwise.
inline std::size_t decodeFrame(const uint8_t* in, std::size_t len, uint8_t* out, std::size_t out_cap) {
  uint8_t raw[kMaxPayloadSize + kCrcSize];
  const std::size_t n = cobsDecode(in, len, raw, sizeof(raw));
  if (n < kCrcSize + 1) return 0;
  const std::size_t payload_len = n - kCrcSize;
  const uint16_t expected = static_cast<uint16_t>(raw[payload_len] | (raw[payload_len + 1] << 8));
  if (crc16(raw, payload_len) != expected) return 0;
  if (payload_len > out_cap) return 0;
  std::memcpy(out, raw, payload_len);
  return payload_len;
}

// Incremental parser. Feed bytes; when feed() returns true, payload()/size()
// describe one validated message until the next feed() call.
class FrameParser {
 public:
  // Returns true when `byte` completed a valid frame.
  bool feed(uint8_t byte) {
    if (byte != kFrameDelimiter) {
      if (fill_ < sizeof(buf_)) {
        buf_[fill_++] = byte;
      } else {
        overflow_ = true;  // frame longer than anything we expect: drop it
      }
      return false;
    }
    bool ok = false;
    if (!overflow_ && fill_ > 0) {
      size_ = decodeFrame(buf_, fill_, payload_, sizeof(payload_));
      ok = size_ > 0;
      if (ok) ++frames_ok_; else ++frames_bad_;
    } else if (overflow_) {
      ++frames_bad_;
    }
    fill_ = 0;
    overflow_ = false;
    return ok;
  }

  const uint8_t* payload() const { return payload_; }
  std::size_t size() const { return size_; }

  // Convenience: true if the last payload is exactly a T with the right type tag.
  template <typename T>
  bool is(MsgType type) const {
    return size_ == sizeof(T) && payload_[0] == kProtocolVersion && payload_[1] == static_cast<uint8_t>(type);
  }

  // Copies the payload into a T. Only valid after is<T>() returned true.
  template <typename T>
  T as() const {
    T msg;
    std::memcpy(&msg, payload_, sizeof(T));
    return msg;
  }

  // Variable-length messages (LogMsg): check header only.
  bool isType(MsgType type) const {
    return size_ >= sizeof(Header) && payload_[0] == kProtocolVersion && payload_[1] == static_cast<uint8_t>(type);
  }

  uint32_t framesOk() const { return frames_ok_; }
  uint32_t framesBad() const { return frames_bad_; }

 private:
  uint8_t buf_[kMaxFrameSize]{};
  uint8_t payload_[kMaxPayloadSize]{};
  std::size_t fill_ = 0;
  std::size_t size_ = 0;
  bool overflow_ = false;
  uint32_t frames_ok_ = 0;
  uint32_t frames_bad_ = 0;
};

}  // namespace arm
