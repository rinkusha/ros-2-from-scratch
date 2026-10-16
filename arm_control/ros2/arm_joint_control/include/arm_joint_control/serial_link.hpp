// Thin POSIX serial port wrapper (termios). Raw 8N1, no flow control.
// Reads are poll()-based so the reader thread can wake up for shutdown.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace arm_joint_control {

class SerialLink {
 public:
  SerialLink() = default;
  ~SerialLink();
  SerialLink(const SerialLink&) = delete;
  SerialLink& operator=(const SerialLink&) = delete;

  // Opens `device` (e.g. /dev/ttyUSB0) at `baud`. Throws std::runtime_error.
  // open()/close() may be called from one thread while write() runs on another.
  // read() must only be used by a single reader thread, which is also the one
  // allowed to call close() on error.
  void open(const std::string& device, int baud);
  void close();
  bool isOpen() const { return fd_.load() >= 0; }

  // Writes all bytes; thread-safe. Returns false on error.
  bool write(const uint8_t* data, std::size_t len);

  // Reads up to `cap` bytes, waiting at most `timeout_ms`. Returns bytes read
  // (0 on timeout), -1 on error/closed.
  int read(uint8_t* buf, std::size_t cap, int timeout_ms);

  // Discards any pending input.
  void flushInput();

  const std::string& device() const { return device_; }

 private:
  std::atomic<int> fd_{-1};
  std::string device_;
  std::mutex write_mutex_;  // also serialises open()/close() against write()
};

}  // namespace arm_joint_control
