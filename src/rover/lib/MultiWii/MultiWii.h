// MultiWii serial protocol
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace MultiWii {

typedef enum {
  DISABLED = 0b00,
  READABLE = 0b01,
  WRITABLE = 0b10,
  BIDIRECTIONAL = READABLE | WRITABLE,
} PacketType;

template <uint8_t CODE, PacketType TYPE, typename PAYLOAD>
class Packet : public PAYLOAD {
public:
  static_assert(sizeof(PAYLOAD) < 255,
                "MSP packet should be less than 255 bytes");
  using Payload = PAYLOAD;
  static inline const PacketType type = TYPE;
  static inline const size_t size = sizeof(Payload);
  static inline const uint8_t code = CODE;
  static inline const char *name;
};

extern const char preamble_send[], preamble_recv[];

typedef enum {
  RECV_PREAMBLE,
  RECV_SIZE,
  RECV_CODE,
  RECV_DATA,
  RECV_CHECKSUM,
} RecvState;

class RawPacket {
public:
  uint8_t size = 0;
  uint8_t code = 0;
  uint8_t checksum = 0;
  std::vector<uint8_t> buffer;

public:
  typedef std::unique_ptr<RawPacket> Ptr;
  // Timestamp when a complete preamble is received
  const std::chrono::system_clock::time_point timestamp;
  RawPacket();

public:
  template <class Packet> inline bool is() {
    static_assert(
        Packet::type & PacketType::READABLE,
        "Protocol does not allow querying this type of packet from device");
    return code == Packet::code;
  }
  // Access to cloned packet data as a typed struct.
  // Throws std::RuntimeException if size mismatches.
  template <class Packet> inline Packet as() {
    static_assert(
        Packet::type & PacketType::READABLE,
        "Protocol does not allow querying this type of packet from device");
    // If the packet passes checksum but still have a mismatched size,
    // it's very likely that our implementation is wrong.
    if (buffer.size() != Packet::size)
      throw std::runtime_error("MultiWii packet " + std::string() +
                               " size mismatch" + ", expected " +
                               std::to_string(Packet::size) + ", received " +
                               std::to_string(buffer.size()));
    Packet packet;
    std::memcpy(&packet, buffer.data(), Packet::size);
    return packet;
  }
};

class NextRawPacket : public RawPacket::Ptr {
public:
  typedef std::unique_ptr<NextRawPacket> Ptr;
  NextRawPacket();
  uint8_t *ptr;
  // Consume one byte until complete
  bool operator()(const char c);
  bool is_complete();
  bool is_good();
};

class Device {
public:
  using Ptr = std::unique_ptr<Device>;
  const int fd;
  Device(const std::string path, int baud = 115200, bool flush = true);
  ~Device();
  // ================================ Outbound ================================
private:
  void write_and_check(std::vector<uint8_t> &buf, const char *name,
                       uint8_t code);

public:
  // Send a query request with no payload
  template <class Packet> void query() {
    static_assert(
        Packet::type & PacketType::READABLE,
        "Protocol does not allow querying this type of packet from device");
    std::vector<uint8_t> outbuf = {'$', 'M', '<', 0, Packet::code};
    uint8_t const &checksum = Packet::code;
    outbuf.push_back(checksum);
    write_and_check(outbuf, Packet::name, Packet::code);
  }
  // Send an action request with payload
  template <class Packet> void send(Packet p) {
    static_assert(
        Packet::type & PacketType::WRITABLE,
        "Protocol does not allow sending this type of packet to device");
    std::vector<uint8_t> outbuf = {'$', 'M', '<', Packet::size, Packet::code};
    outbuf.reserve(Packet::size + 6);
    uint8_t checksum = Packet::size ^ Packet::code;
    const auto &d = reinterpret_cast<uint8_t *>(&p);
    for (uint8_t *p = d; p < d + Packet::size; p++) {
      outbuf.push_back(*p);
      checksum ^= *p;
    }
    outbuf.push_back(checksum);
    write_and_check(outbuf, Packet::name, Packet::code);
  }
  // ================================ Inbound ================================
private:
  std::mutex mutex;
  std::queue<RawPacket::Ptr> queue;

  const std::thread recv_thread;
  bool flag_term = false;
  NextRawPacket::Ptr next_packet = nullptr;
  const char *preamble = preamble_recv;
  // Standalone packet receiver
  void receiver();

public:
  // Non-blocking, returns nullptr if no packet is available
  RawPacket::Ptr read();
};

} // namespace MultiWii
