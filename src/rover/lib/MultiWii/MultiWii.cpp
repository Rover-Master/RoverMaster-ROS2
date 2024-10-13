// MultiWii serial protocol
#include "MultiWii.h"

#include "serial/serial.h"

#include <iostream>
#include <sys/types.h>

using namespace std::chrono_literals;

namespace MultiWii {

RawPacket::RawPacket() : timestamp(std::chrono::system_clock::now()) {}

NextRawPacket::NextRawPacket()
    : RawPacket::Ptr(std::make_unique<RawPacket>()),
      ptr(reinterpret_cast<uint8_t *>(this->get())) {}

bool NextRawPacket::operator()(const char c) {
  auto &packet = **this;
  packet.checksum ^= c;
  if (ptr == &packet.checksum) {
    if (packet.buffer.size() < packet.size)
      packet.buffer.push_back(c);
    else
      return true;
  } else {
    *ptr = c;
    ptr++;
  }
  return false;
}

bool NextRawPacket::is_good() {
  const auto &packet = **this;
  return packet.checksum == 0;
}

// http://www.multiwii.com/wiki/index.php
const char preamble_send[] = "$M<";
const char preamble_recv[] = "$M>";

Device::Device(const std::string path, int baud, bool flush)
    // Check of fd has been done in serial::open
    : fd(serial::open(path.c_str(), baud)),
      recv_thread(&Device::receiver, this) {
  if (flush) {
    std::this_thread::sleep_for(100ms);
    serial::flush(fd);
    std::this_thread::sleep_for(100ms);
  }
};

Device::~Device() {
  flag_term = true;
  recv_thread.join();
  close(fd);
}

void Device::write_and_check(std::vector<uint8_t> &buf, const char *name,
                             uint8_t code) {
  const auto ret = write(fd, buf.data(), buf.size());
  if (ret < 0 || static_cast<unsigned long>(ret) != buf.size())
    throw std::runtime_error(
        "MultiWii: failed to send packet " + std::string(name) + " (code " +
        std::to_string(code) + ")" +
        ", length = " + std::to_string(buf.size()) +
        ", sent = " + std::to_string(ret) + ", error: " + strerror(errno));
}

void Device::receiver() {
  uint8_t recv_buffer[256];
  uint8_t *ptr;
  while (!flag_term) {
    const auto ret = ::read(fd, recv_buffer, sizeof(recv_buffer));
    if (ret < 0) {
      std::cerr << "MultiWii: failed to read from device: " << strerror(errno)
                << ", aborting ..." << std::endl;
      break;
    }
    if (ret == 0)
      continue;
    ptr = recv_buffer;
    while (ptr < recv_buffer + ret) {
      if (next_packet != nullptr) {
        // Direct data to already existing next raw packet
        const auto complete = (*next_packet)(*ptr);
        if (complete) {
          if (next_packet->is_good()) {
            std::lock_guard<std::mutex> lock(mutex);
            queue.push(std::move(*next_packet));
          } else {
            std::cerr << "MultiWii: bad packet received" << std::endl;
          }
          next_packet = nullptr;
        }
      } else if (*preamble == '\0') {
        // Initialize next raw packet
        next_packet = std::make_unique<NextRawPacket>();
        preamble = preamble_recv;
        // Do not increment ptr
        continue;
      } else {
        // Match preamble
        if (*ptr == *preamble)
          preamble++;
        else
          preamble = preamble_recv;
      }
      ptr++;
    }
  }
}

RawPacket::Ptr Device::read() {
  std::lock_guard<std::mutex> lock(mutex);
  if (queue.empty())
    return nullptr;
  auto packet = std::move(queue.front());
  queue.pop();
  return packet;
}

} // namespace MultiWii
