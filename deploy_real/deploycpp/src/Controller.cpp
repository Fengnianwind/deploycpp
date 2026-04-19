#include "Controller.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "ControllerSafety.h"

namespace {
const char* stateName(FSMStateName state) {
  switch (state) {
    case FSMStateName::Passive:
      return "Passive";
    case FSMStateName::FixedPose:
      return "FixedPose";
    case FSMStateName::LocoMode:
      return "LocoMode";
    case FSMStateName::BeyondMimic:
      return "BeyondMimic";
    case FSMStateName::BeyondMimic2:
      return "BeyondMimic2";
  }
  return "Unknown";
}
}  // namespace

Controller::Controller(const Config& config)
    : config_(config),
      robot_io_(config),
      fsm_(config, true) {
  output_.setDamping(8.0f);
}

int Controller::run() {
  const auto cycle = std::chrono::duration<double>(config_.control_dt);
  std::cout << "state=Passive\n";

  while (!robot_io_.hasLowState()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  bool running = true;
  while (running) {
    const auto loop_start = std::chrono::steady_clock::now();
    StateAndCmd state = robot_io_.latestStateAndCmd();

    if (state.remote.select) {
      std::cout << "select_exit=1\n";
      sendDamping();
      std::cout << "exit\n";
      return 0;
    }

    const FSMStateName before = fsm_.currentState();
    FSMStateName after = before;
    runWithDampingFallback(
        [&]() {
          fsm_.run(state, output_);
          after = fsm_.currentState();

          if (!output_.isFinite()) {
            std::cerr << "invalid_policy_output=1\n";
            output_.setDamping(8.0f);
          }

          robot_io_.sendCommand(output_);
        },
        [&]() { sendDamping(); });

    if (before != after) {
      if (before == FSMStateName::Passive && after == FSMStateName::FixedPose) {
        std::cout << "transition=Passive->FixedPose\n";
        fixed_pose_complete_reported_ = false;
      } else if (before == FSMStateName::FixedPose && after == FSMStateName::LocoMode) {
        std::cout << "transition=FixedPose->LocoMode\n";
      } else if (before == FSMStateName::FixedPose && after == FSMStateName::BeyondMimic) {
        std::cout << "transition=FixedPose->BeyondMimic\n";
      } else if (before == FSMStateName::FixedPose && after == FSMStateName::BeyondMimic2) {
        std::cout << "transition=FixedPose->BeyondMimic2\n";
      } else if (before == FSMStateName::LocoMode && after == FSMStateName::BeyondMimic) {
        std::cout << "transition=LocoMode->BeyondMimic\n";
      } else if (before == FSMStateName::LocoMode && after == FSMStateName::BeyondMimic2) {
        std::cout << "transition=LocoMode->BeyondMimic2\n";
      } else if (before == FSMStateName::BeyondMimic && after == FSMStateName::LocoMode) {
        std::cout << "transition=BeyondMimic->LocoMode\n";
      } else if (before == FSMStateName::BeyondMimic2 && after == FSMStateName::LocoMode) {
        std::cout << "transition=BeyondMimic2->LocoMode\n";
      } else if (after == FSMStateName::Passive) {
        std::cout << "transition=" << stateName(before) << "->Passive\n";
      }
    }
    if (after == FSMStateName::FixedPose && fsm_.fixedPoseState().complete() && !fixed_pose_complete_reported_) {
      std::cout << "fixed_pose_step=" << fsm_.fixedPoseState().currentStep() << "/"
                << fsm_.fixedPoseState().totalSteps() << "\n";
      std::cout << "fixed_pose_complete=1\n";
      fixed_pose_complete_reported_ = true;
    }

    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < cycle) {
      std::this_thread::sleep_for(cycle - elapsed);
    } else {
      std::cout << "control_loop_overrun=1\n";
    }
  }

  return 0;
}

void Controller::sendDamping() {
  output_.setDamping(8.0f);
  robot_io_.sendCommand(output_);
  std::cout << "damping_sent=1\n";
}
