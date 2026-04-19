#include <iostream>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "robot/CommandUtils.h"
#include "types/PolicyOutput.h"

int main() {
  unitree_hg::msg::dds_::LowState_ state;
  state.mode_machine() = 7;

  PolicyOutput output;
  output.setDamping(8.0f);

  unitree_hg::msg::dds_::LowCmd_ cmd;
  buildLowCommand(cmd, state, output);

  if (cmd.mode_machine() != 7) {
    std::cerr << "mode_machine mismatch\n";
    return 1;
  }
  if (cmd.mode_pr() != 0) {
    std::cerr << "mode_pr mismatch\n";
    return 1;
  }

  for (int i = 0; i < 29; ++i) {
    const auto& motor = cmd.motor_cmd()[i];
    if (motor.mode() != 1 || motor.q() != 0.0f || motor.dq() != 0.0f ||
        motor.kp() != 0.0f || motor.kd() != 8.0f || motor.tau() != 0.0f) {
      std::cerr << "motor command mismatch at " << i << "\n";
      return 1;
    }
  }

  if (cmd.crc() == 0) {
    std::cerr << "crc was not populated\n";
    return 1;
  }

  std::cout << "command_utils_test_pass=1\n";
  return 0;
}
