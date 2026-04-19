#include "launcher/LauncherStateMachine.h"

namespace {

bool isArmComboPressed(const RemoteState& remote) {
  return remote.L2 && remote.R2;
}

}  // namespace

const char* launcherStateName(LauncherState state) {
  switch (state) {
    case LauncherState::Idle:
      return "Idle";
    case LauncherState::Armed:
      return "Armed";
    case LauncherState::Starting:
      return "Starting";
    case LauncherState::Running:
      return "Running";
    case LauncherState::Backoff:
      return "Backoff";
    case LauncherState::Locked:
      return "Locked";
  }
  return "Unknown";
}

LauncherStateMachine::LauncherStateMachine(LauncherPolicy policy)
    : policy_(std::move(policy)) {}

LauncherStepResult LauncherStateMachine::tick(const RemoteState& remote, bool low_state_connected, TimePoint now) {
  const bool arm_combo = isArmComboPressed(remote);
  const bool arm_rising = arm_combo && !previous_arm_combo_;
  const bool x_rising = remote.X && !previous_x_;

  LauncherStepResult result;
  switch (state_) {
    case LauncherState::Idle:
      if (low_state_connected && arm_rising && runner_pid_ <= 0) {
        arm(now);
      }
      break;
    case LauncherState::Armed:
      if (!low_state_connected) {
        state_ = LauncherState::Idle;
      } else if (armed_deadline_ && now >= *armed_deadline_) {
        state_ = LauncherState::Idle;
      } else if (x_rising) {
        state_ = LauncherState::Starting;
        result.start_runner = true;
      }
      break;
    case LauncherState::Starting:
      if (!low_state_connected) {
        state_ = LauncherState::Idle;
        result.stop_runner = true;
      }
      break;
    case LauncherState::Running:
      if (!low_state_connected) {
        state_ = LauncherState::Idle;
        result.stop_runner = true;
        resetRestartBudget();
      }
      break;
    case LauncherState::Backoff:
      if (!low_state_connected) {
        state_ = LauncherState::Idle;
        resetRestartBudget();
      } else if (backoff_deadline_ && now >= *backoff_deadline_) {
        state_ = LauncherState::Starting;
        result.start_runner = true;
      }
      break;
    case LauncherState::Locked:
      if (low_state_connected && arm_rising && runner_pid_ <= 0) {
        resetRestartBudget();
        arm(now);
      }
      break;
  }

  previous_arm_combo_ = arm_combo;
  previous_x_ = remote.X;
  return result;
}

void LauncherStateMachine::onRunnerSpawned(int pid) {
  runner_pid_ = pid;
  state_ = LauncherState::Running;
}

void LauncherStateMachine::onRunnerExit(int exit_code, Duration runtime, bool low_state_connected, TimePoint now) {
  runner_pid_ = -1;
  current_backoff_ = Duration::zero();
  backoff_deadline_.reset();

  if (exit_code == 0 || !low_state_connected) {
    state_ = LauncherState::Idle;
    resetRestartBudget();
    return;
  }

  if (runtime >= policy_.fast_failure_threshold ||
      !failure_window_start_ ||
      now - *failure_window_start_ > policy_.failure_window) {
    failure_window_start_ = now;
    restart_attempt_ = 0;
  }

  ++restart_attempt_;
  if (restart_attempt_ > static_cast<int>(policy_.backoff_schedule.size())) {
    state_ = LauncherState::Locked;
    return;
  }

  current_backoff_ = policy_.backoff_schedule[static_cast<std::size_t>(restart_attempt_ - 1)];
  backoff_deadline_ = now + current_backoff_;
  state_ = LauncherState::Backoff;
}

LauncherState LauncherStateMachine::state() const {
  return state_;
}

int LauncherStateMachine::restart_attempt() const {
  return restart_attempt_;
}

LauncherStateMachine::Duration LauncherStateMachine::current_backoff() const {
  return current_backoff_;
}

int LauncherStateMachine::runner_pid() const {
  return runner_pid_;
}

void LauncherStateMachine::arm(TimePoint now) {
  state_ = LauncherState::Armed;
  armed_deadline_ = now + policy_.arm_timeout;
}

void LauncherStateMachine::resetRestartBudget() {
  restart_attempt_ = 0;
  current_backoff_ = Duration::zero();
  armed_deadline_.reset();
  backoff_deadline_.reset();
  failure_window_start_.reset();
}
