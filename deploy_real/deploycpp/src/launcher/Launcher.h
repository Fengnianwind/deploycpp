#pragma once

#include <chrono>
#include <string>

#include "RobotIO.h"
#include "config/Config.h"
#include "launcher/LauncherStateMachine.h"

class Launcher {
 public:
  Launcher(const Config& config, std::string executable_path, std::string project_root);

  int run();

 private:
  using Clock = std::chrono::steady_clock;

  bool spawnRunner(Clock::time_point now);
  void terminateRunner();
  void pollRunner(bool low_state_connected, Clock::time_point now);
  void logStateIfChanged(LauncherState before) const;
  void logLowStateIfChanged(bool connected);

  Config config_;
  RobotIO robot_io_;
  LauncherStateMachine machine_;
  std::string executable_path_;
  std::string project_root_;
  int runner_pid_ = -1;
  Clock::time_point runner_started_at_{};
  bool reported_low_state_connected_ = false;
  bool reported_low_state_initialized_ = false;
};
