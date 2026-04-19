#include "robot/RemoteControl.h"

#include <cstring>

namespace {

float readFloat(const uint8_t* data, std::size_t offset) {
  float value = 0.0f;
  std::memcpy(&value, data + offset, sizeof(float));
  return value;
}

bool bit(uint16_t keys, int index) {
  return ((keys >> index) & 0x1U) != 0;
}

}  // namespace

RemoteState decodeRemoteState(const uint8_t* data, std::size_t size) {
  if (data == nullptr || size < 24) {
    return {};
  }

  const uint16_t keys = static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8U);

  RemoteState remote;
  remote.R1 = bit(keys, 0);
  remote.L1 = bit(keys, 1);
  remote.start = bit(keys, 2);
  remote.select = bit(keys, 3);
  remote.R2 = bit(keys, 4);
  remote.L2 = bit(keys, 5);
  remote.F1 = bit(keys, 6);
  remote.A = bit(keys, 8);
  remote.B = bit(keys, 9);
  remote.X = bit(keys, 10);
  remote.Y = bit(keys, 11);
  remote.lx = readFloat(data, 4);
  remote.rx = readFloat(data, 8);
  remote.ry = readFloat(data, 12);
  remote.ly = readFloat(data, 20);
  return remote;
}
