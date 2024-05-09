#include "serial.h"
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <termios.h>

#define CHECK(RET)                                                             \
  {                                                                            \
    if ((RET) < 0)                                                             \
      throw std::runtime_error(strerror(errno));                               \
  }

void set_interface_attribs(int fd, int speed) {
  struct termios tty;
  CHECK(tcgetattr(fd, &tty))
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  cfmakeraw(&tty);
  // no remapping, no delays
  tty.c_oflag = 0;
  // Set the attributes
  CHECK(tcsetattr(fd, TCSANOW, &tty))
}

void set_blocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
    throw std::runtime_error(strerror(errno));
  tty.c_cc[VMIN] = should_block ? 1 : 0;
  // 0.5 seconds read timeout
  tty.c_cc[VTIME] = should_block ? 1 : 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    throw std::runtime_error(strerror(errno));
}

const auto _open = open;
const auto _write = write;

int serial::open(const char *port, int baud, int blocking) {
  int fd = _open(port, O_RDWR | O_NOCTTY | O_SYNC);
  CHECK(fd);
  // set speed to baud, 8n1 (no parity)
  set_interface_attribs(fd, baud);
  // set no blocking
  set_blocking(fd, blocking);
  return fd;
}

int serial::flush(int fd) { return tcflush(fd, TCIOFLUSH); }

int serial::write(int fd, std::string data) {
  return _write(fd, data.c_str(), data.size());
}

int serial::readline(int fd, char **cursor, int size, const char delimiter) {
  return serial::readline(fd, cursor, *cursor + size, delimiter);
};

int serial::readline(int fd, char **cursor, char *const line_ep,
                     const char delimiter) {
  while (1) {
    if (*cursor == line_ep) {
      // Buffer Overflow
      return -1;
    }
    if (read(fd, *cursor, 1) > 0) {
      if ((**cursor) == delimiter) {
        // Replace delimiter with null character
        **cursor = '\0';
        // Report line ready
        return 1;
      } else {
        // Advance cursor, wait for next character
        (*cursor)++;
      }
    } else {
      // No more data available
      return 0;
    }
  };
}
