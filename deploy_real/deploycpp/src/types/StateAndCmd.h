#pragma once

#include <array>

#include "types/RemoteState.h"

enum class FSMCommand {
  Invalid,
  Passive,
  LocoMode,
  FixedPose,
  BeyondMimic2,
  BeyondMimic,
  Exit,
};

struct StateAndCmd {
  std::array<float, 29> q{};
  std::array<float, 29> dq{};
  std::array<float, 4> base_quat{1.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 3> ang_vel{};
  std::array<float, 3> vel_cmd{};
  RemoteState remote;
  FSMCommand skill_cmd = FSMCommand::Invalid;
};
