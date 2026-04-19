#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "robot/RemoteControl.h"

namespace {

void writeFloat(std::array<uint8_t, 24>& data, std::size_t offset, float value) {
  std::memcpy(data.data() + offset, &value, sizeof(float));
}

bool approx(float lhs, float rhs, float tolerance = 1e-6f) {
  return lhs >= rhs - tolerance && lhs <= rhs + tolerance;
}

}  // namespace

int main() {
  std::array<uint8_t, 24> raw{};
  const uint16_t keys = static_cast<uint16_t>((1U << 0U) | (1U << 4U) | (1U << 5U) | (1U << 10U));
  raw[2] = static_cast<uint8_t>(keys & 0xffU);
  raw[3] = static_cast<uint8_t>((keys >> 8U) & 0xffU);
  writeFloat(raw, 4, 0.25f);
  writeFloat(raw, 8, -0.5f);
  writeFloat(raw, 12, 0.75f);
  writeFloat(raw, 20, -1.0f);

  const RemoteState remote = decodeRemoteState(raw.data(), raw.size());
  if (!remote.R1 || !remote.R2 || !remote.L2 || !remote.X) {
    std::cerr << "expected R1/R2/L2/X bits to decode\n";
    return 1;
  }
  if (remote.L1 || remote.start || remote.select || remote.F1 || remote.A || remote.B || remote.Y) {
    std::cerr << "unexpected extra buttons decoded\n";
    return 1;
  }
  if (!approx(remote.lx, 0.25f) || !approx(remote.rx, -0.5f) ||
      !approx(remote.ry, 0.75f) || !approx(remote.ly, -1.0f)) {
    std::cerr << "axis decode mismatch\n";
    return 1;
  }

  const RemoteState empty = decodeRemoteState(raw.data(), 8);
  if (empty.R1 || empty.R2 || empty.L2 || empty.X) {
    std::cerr << "short payload should decode to default remote state\n";
    return 1;
  }

  std::cout << "remote_control_test_pass=1\n";
  return 0;
}
