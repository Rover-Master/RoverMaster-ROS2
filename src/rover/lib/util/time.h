#pragma once

#include <chrono>

namespace Time {

static inline std::chrono::time_point<std::chrono::system_clock> now() {
  return std::chrono::high_resolution_clock::now();
}
// Get current timestamp in milliseconds
static inline unsigned long ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

// Get current timestamp in nanoseconds
static inline unsigned long us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

// Get current timestamp in nanoseconds
static inline unsigned long ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

// Get current time relative to origin in milliseconds
static inline unsigned long
ms(std::chrono::time_point<std::chrono::system_clock> origin) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - origin)
      .count();
}

// Get current time relative to origin in nanoseconds
static inline unsigned long
us(std::chrono::time_point<std::chrono::system_clock> origin) {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now() - origin)
      .count();
}

// Get current time relative to origin in nanoseconds
static inline unsigned long
ns(std::chrono::time_point<std::chrono::system_clock> origin) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::high_resolution_clock::now() - origin)
      .count();
}

} // namespace Time
