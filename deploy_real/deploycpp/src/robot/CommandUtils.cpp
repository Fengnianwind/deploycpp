#include "robot/CommandUtils.h"

#include <cstdint>

unsigned int crc32Core(unsigned int* ptr, unsigned int len) {
  unsigned int xbit = 0;
  unsigned int data = 0;
  unsigned int crc32 = 0xFFFFFFFF;
  const unsigned int polynomial = 0x04c11db7;
  for (unsigned int i = 0; i < len; i++) {
    xbit = 1U << 31U;
    data = ptr[i];
    for (unsigned int bits = 0; bits < 32; bits++) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1U;
        crc32 ^= polynomial;
      } else {
        crc32 <<= 1U;
      }
      if (data & xbit) {
        crc32 ^= polynomial;
      }
      xbit >>= 1U;
    }
  }
  return crc32;
}

void buildLowCommand(
    unitree_hg::msg::dds_::LowCmd_& cmd,
    const unitree_hg::msg::dds_::LowState_& state,
    const PolicyOutput& output) {
  cmd.mode_machine() = state.mode_machine();
  cmd.mode_pr() = 0;
  for (std::size_t i = 0; i < output.actions.size(); ++i) {
    auto& motor = cmd.motor_cmd()[i];
    motor.mode() = 1;
    motor.q() = output.actions[i];
    motor.dq() = 0.0f;
    motor.kp() = output.kps[i];
    motor.kd() = output.kds[i];
    motor.tau() = 0.0f;
  }
  cmd.crc() = crc32Core(reinterpret_cast<uint32_t*>(&cmd), (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2U) - 1U);
}
