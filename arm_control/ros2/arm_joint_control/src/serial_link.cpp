#include "arm_joint_control/serial_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace arm_joint_control {

namespace {

speed_t toSpeed(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: throw std::runtime_error("unsupported baud rate " + std::to_string(baud));
  }
}

}  // namespace

SerialLink::~SerialLink() { close(); }

void SerialLink::open(const std::string& device, int baud) {
  close();
  const speed_t speed = toSpeed(baud);
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("open(" + device + ") failed: " + std::strerror(errno));
  }

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    const int err = errno;
    ::close(fd);
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(err)));
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;  // no HW flow control
  tio.c_cflag &= ~CSTOPB;   // 1 stop bit
  tio.c_cflag &= ~PARENB;   // no parity
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    const int err = errno;
    ::close(fd);
    throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(err)));
  }
  tcflush(fd, TCIOFLUSH);

  std::lock_guard<std::mutex> lock(write_mutex_);
  fd_.store(fd);
  device_ = device;
}

void SerialLink::close() {
  std::lock_guard<std::mutex> lock(write_mutex_);
  const int fd = fd_.exchange(-1);
  if (fd >= 0) ::close(fd);
}

bool SerialLink::write(const uint8_t* data, std::size_t len) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  const int fd = fd_.load();
  if (fd < 0) return false;
  std::size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::write(fd, data + sent, len - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        pollfd pfd{fd, POLLOUT, 0};
        if (::poll(&pfd, 1, 50) <= 0) return false;
        continue;
      }
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

int SerialLink::read(uint8_t* buf, std::size_t cap, int timeout_ms) {
  const int fd = fd_.load();
  if (fd < 0) return -1;
  pollfd pfd{fd, POLLIN, 0};
  const int r = ::poll(&pfd, 1, timeout_ms);
  if (r < 0) return (errno == EINTR) ? 0 : -1;
  if (r == 0) return 0;
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
  const ssize_t n = ::read(fd, buf, cap);
  if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
  if (n == 0) return -1;  // EOF: device went away
  return static_cast<int>(n);
}

void SerialLink::flushInput() {
  const int fd = fd_.load();
  if (fd >= 0) tcflush(fd, TCIFLUSH);
}

}  // namespace arm_joint_control
