// MultiWii serial protocol
#include "MultiWii.h"

namespace MultiWii {

// http://www.multiwii.com/wiki/index.php
const char preamble_send[] = "$M<";
const char preamble_recv[] = "$M>";
// Memory view
// struct {
//   uint8_t preamble[3]; // "$M<" (send) or "$M>" (recv)
//   uint8_t size;
//   uint8_t code;
//   uint8_t data[size];
//   uint8_t checksum; // size ^ code ^ data[0] ^ ... ^ data[size - 1];
// } MultiWiiPacket;

bool Receiver::recv(const char c) {
  if (state != RECV_PREAMBLE)
    packet.checksum ^= c;
  switch (state) {
  case RECV_SIZE:
    packet.size = c;
    state = RECV_CODE;
    break;
  case RECV_CODE:
    packet.code = c;
    if (packet.size == 0)
      state = RECV_CHECKSUM;
    else
      state = RECV_DATA;
    break;
  case RECV_DATA:
    buffer.push_back(c);
    if (buffer.size() == packet.size) {
      state = RECV_CHECKSUM;
    }
    break;
  case RECV_CHECKSUM:
    state = RECV_PREAMBLE;
    return packet.checksum == 0;
  case RECV_PREAMBLE:
  default:
    if (c != '\0' && c == *preamble)
      preamble++;
    else
      preamble = preamble_recv;
    if (*preamble == '\0') {
      packet.checksum = 0;
      buffer.clear();
      preamble = preamble_recv;
      state = RECV_SIZE;
    }
    break;
  }
  return false;
}

} // namespace MultiWii
