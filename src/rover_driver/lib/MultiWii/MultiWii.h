// MultiWii serial protocol
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace MultiWii {

extern const char preamble_send[], preamble_recv[];

static inline void write_and_check(const int fd, const char *name,
                                   const uint8_t code,
                                   std::vector<uint8_t> &buf) {
  const auto ret = write(fd, buf.data(), buf.size());
  if (ret < 0 || static_cast<unsigned long>(ret) != buf.size())
    throw std::runtime_error(
        "MultiWii: failed to send packet " + std::string(name) + " (code " +
        std::to_string(code) + ")" +
        ", length = " + std::to_string(buf.size()) +
        ", sent = " + std::to_string(ret) + ", error: " + strerror(errno));
}

// Send a query request with no payload
template <const char *name, uint8_t code, typename T>
static inline void send(int fd) {
  static_assert(sizeof(T) < 255, "MSP data should be less than 255 bytes");
  std::vector<uint8_t> outbuf = {'$', 'M', '<', 0, code};
  uint8_t const &checksum = code;
  outbuf.push_back(checksum);
  write_and_check(fd, name, code, outbuf);
}

// Send an action request with payload
template <const char *name, uint8_t code, typename T>
static inline void send(int fd, T data) {
  static_assert(sizeof(T) < 255, "MSP data should be less than 255 bytes");
  std::vector<uint8_t> outbuf = {'$', 'M', '<', sizeof(T), code};
  uint8_t checksum = sizeof(T) ^ code;
  const auto &d = reinterpret_cast<uint8_t *>(&data);
  for (uint8_t *p = d; p < d + sizeof(T); p++) {
    outbuf.push_back(*p);
    checksum ^= *p;
  }
  outbuf.push_back(checksum);
  write_and_check(fd, name, code, outbuf);
}

typedef enum {
  RECV_PREAMBLE,
  RECV_SIZE,
  RECV_CODE,
  RECV_DATA,
  RECV_CHECKSUM,
} RecvState;

class Receiver {
private:
  struct {
    uint8_t size = 0;
    uint8_t code = 0;
    uint8_t checksum = 0;
  } packet;
  std::vector<uint8_t> buffer;
  const char *preamble = preamble_recv;
  RecvState state = RECV_PREAMBLE;

public:
  // Process next byte from the serial port.
  // Returns TRUE  if the packet is complete and checksum is correct.
  // Returns FALSE if the packet is not yet complete.
  bool recv(const char c);
  // Returns TRUE  if the packet matches the expected code.
  // Returns FALSE otherwise.
  template <const char *name, uint8_t code, typename T> inline bool match() {
    return packet.code == code;
  }
  // Access to cloned packet data as a typed struct.
  // Throws std::RuntimeException if size mismatches.
  template <const char *name, uint8_t code, typename T> inline T data() {
    if (buffer.size() != sizeof(T))
      throw std::runtime_error("MultiWii packet " + std::string(name) +
                               " size mismatch" + ", expected " +
                               std::to_string(sizeof(T)) + ", received " +
                               std::to_string(buffer.size()));
    T ret;
    std::memcpy(&ret, buffer.data(), buffer.size());
    return ret;
  }
};

} // namespace MultiWii
