#pragma once

#include <chrono>
#include <optional>
#include <vector>

#include "types/RemoteState.h"

enum class LauncherState {
  Idle,
  Armed,
  Starting,
  Running,
  Backoff,
  Locked,
};

struct LauncherPolicy {
  std::chrono::steady_clock::duration arm_timeout = std::chrono::seconds(5);
  std::chrono::steady_clock::duration failure_window = std::chrono::seconds(60);
  std::chrono::steady_clock::duration fast_failure_threshold = std::chrono::seconds(2);
  std::vector<std::chrono::steady_clock::duration> backoff_schedule{
      std::chrono::seconds(1),
      std::chrono::seconds(3),
      std::chrono::seconds(10),
  };
};

struct LauncherStepResult {
  bool start_runner = false;
  bool stop_runner = false;
};

const char* launcherStateName(LauncherState state);

class LauncherStateMachine {
 public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  explicit LauncherStateMachine(LauncherPolicy policy = {});

  LauncherStepResult tick(const RemoteState& remote, bool low_state_connected, TimePoint now);
  void onRunnerSpawned(int pid);
  void onRunnerExit(int exit_code, Duration runtime, bool low_state_connected, TimePoint now);

  LauncherState state() const;
  int restart_attempt() const;
  Duration current_backoff() const;
  int runner_pid() const;

 private:
  void arm(TimePoint now);
  void resetRestartBudget();

  LauncherPolicy policy_;
  LauncherState state_ = LauncherState::Idle;
  bool previous_arm_combo_ = false;
  bool previous_x_ = false;
  int restart_attempt_ = 0;
  int runner_pid_ = -1;
  Duration current_backoff_{};
  std::optional<TimePoint> armed_deadline_;
  std::optional<TimePoint> backoff_deadline_;
  std::optional<TimePoint> failure_window_start_;
};
