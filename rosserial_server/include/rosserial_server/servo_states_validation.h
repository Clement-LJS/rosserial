// Validate the fixed spinal/ServoStates wire schema before any typed allocation.
#ifndef ROSSERIAL_SERVER_SERVO_STATES_VALIDATION_H
#define ROSSERIAL_SERVER_SERVO_STATES_VALIDATION_H

#include <cstddef>
#include <cstdint>

namespace rosserial_server
{
inline uint32_t servoStatesReadLE32(const uint8_t* p)
{
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

// Returns nullptr for a valid packet, otherwise a diagnostic reason. No ROS
// message is constructed and count is never used to allocate memory.
inline const char* validateServoStates(const uint8_t* data, size_t length)
{
  const size_t header_size = 12;  // seconds, nanoseconds, array count
  const size_t record_size = 7;  // index, angle, temperature, load, error
  if (length < header_size) return "truncated header";
  const uint32_t count = servoStatesReadLE32(data + 8);
  // Divide the actual buffer size, avoiding multiplication overflow from count.
  if ((length - header_size) % record_size != 0 ||
      count != (length - header_size) / record_size)
    return "array count does not match payload length";
  if (servoStatesReadLE32(data + 4) >= 1000000000u)
    return "invalid timestamp nanoseconds";
  if (count > 256) return "too many servo indices";
  bool seen[256] = {};
  for (size_t i = 0; i < count; ++i)
  {
    const uint8_t index = data[header_size + record_size * i];
    if (seen[index]) return "duplicate servo index";
    seen[index] = true;
  }
  // Do not reject hardware error bits: a real servo fault must still be reported.
  return nullptr;
}
}  // namespace rosserial_server
#endif
