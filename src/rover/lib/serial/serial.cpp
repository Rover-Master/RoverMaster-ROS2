#include "serial.h"
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdexcept>
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

#include <libudev.h>

std::vector<serial::PortInfo> serial::locate(std::string vid, std::string pid) {
  struct udev *udev = udev_new();
  if (!udev)
    throw std::runtime_error("udev instance cannot be created");
  std::vector<serial::PortInfo> ports;
  struct udev_enumerate *enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "tty");
  udev_enumerate_scan_devices(enumerate);

  struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
  struct udev_list_entry *dev_list_entry;
  udev_list_entry_foreach(dev_list_entry, devices) {
    const char *path = udev_list_entry_get_name(dev_list_entry);
    struct udev_device *dev = udev_device_new_from_syspath(udev, path);
    struct udev_device *parent =
        udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    if (parent) {
      const char *device_vid =
          udev_device_get_sysattr_value(parent, "idVendor");
      const char *device_pid =
          udev_device_get_sysattr_value(parent, "idProduct");
      if (!vid.empty() && vid != device_vid)
        continue;
      if (!pid.empty() && pid != device_pid)
        continue;
      ports.push_back(serial::PortInfo{udev_device_get_devnode(dev), device_vid,
                                       device_pid});
    }
    udev_device_unref(dev);
  }
  udev_enumerate_unref(enumerate);
  udev_unref(udev);
  return ports;
}
