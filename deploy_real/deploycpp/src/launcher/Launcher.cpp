#include "launcher/Launcher.h"

#include <csignal>
#include <iostream>
#include <thread>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

Launcher::Launcher(const Config& config, std::string executable_path, std::string project_root)
    : config_(config),
      robot_io_(config),
      machine_(LauncherPolicy{}),
      executable_path_(std::move(executable_path)),
      project_root_(std::move(project_root)) {}

int Launcher::run() {
  const auto cycle = std::chrono::duration<double>(config_.control_dt);
  std::cout << "launcher_state=" << launcherStateName(machine_.state()) << '\n';

  while (true) {
    const auto loop_start = Clock::now();
    const bool low_state_connected = robot_io_.hasLowState();
    logLowStateIfChanged(low_state_connected);
    pollRunner(low_state_connected, loop_start);

    RemoteState remote;
    if (low_state_connected) {
      remote = robot_io_.latestStateAndCmd().remote;
    }

    const LauncherState before = machine_.state();
    const LauncherStepResult step = machine_.tick(remote, low_state_connected, loop_start);
    logStateIfChanged(before);

    if (step.stop_runner) {
      std::cout << "lock_reason=low_state_lost\n";
      terminateRunner();
    }

    if (step.start_runner) {
      std::cout << "start_trigger=x_rising_edge\n";
      const LauncherState before_spawn = machine_.state();
      if (spawnRunner(loop_start)) {
        machine_.onRunnerSpawned(runner_pid_);
        std::cout << "runner_pid=" << runner_pid_ << '\n';
      } else {
        machine_.onRunnerExit(127, std::chrono::seconds(0), low_state_connected, loop_start);
        std::cout << "runner_exit_code=127\n";
      }
      logStateIfChanged(before_spawn);
      if (machine_.state() == LauncherState::Backoff) {
        std::cout << "restart_attempt=" << machine_.restart_attempt() << '\n';
        std::cout << "backoff_seconds="
                  << std::chrono::duration_cast<std::chrono::seconds>(machine_.current_backoff()).count() << '\n';
      } else if (machine_.state() == LauncherState::Locked) {
        std::cout << "lock_reason=too_many_failures\n";
      }
    } else if (before == LauncherState::Idle && machine_.state() == LauncherState::Armed) {
      std::cout << "arm_reason=l2_r2_combo\n";
    }

    const auto elapsed = Clock::now() - loop_start;
    if (elapsed < cycle) {
      std::this_thread::sleep_for(cycle - elapsed);
    }
  }
}

bool Launcher::spawnRunner(Clock::time_point now) {
  const pid_t pid = fork();
  if (pid < 0) {
    return false;
  }
  if (pid == 0) {
    execl(executable_path_.c_str(), executable_path_.c_str(), "--run", project_root_.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }

  runner_pid_ = static_cast<int>(pid);
  runner_started_at_ = now;
  return true;
}

void Launcher::terminateRunner() {
  if (runner_pid_ > 0) {
    kill(runner_pid_, SIGTERM);
  }
}

void Launcher::pollRunner(bool low_state_connected, Clock::time_point now) {
  if (runner_pid_ <= 0) {
    return;
  }

  int status = 0;
  const pid_t wait_result = waitpid(static_cast<pid_t>(runner_pid_), &status, WNOHANG);
  if (wait_result <= 0) {
    return;
  }

  int exit_code = 1;
  if (WIFEXITED(status)) {
    exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    exit_code = 128 + WTERMSIG(status);
  }

  const auto runtime = now - runner_started_at_;
  runner_pid_ = -1;
  const LauncherState before = machine_.state();
  machine_.onRunnerExit(exit_code, runtime, low_state_connected, now);
  std::cout << "runner_exit_code=" << exit_code << '\n';
  logStateIfChanged(before);

  if (machine_.state() == LauncherState::Backoff) {
    std::cout << "restart_attempt=" << machine_.restart_attempt() << '\n';
    std::cout << "backoff_seconds="
              << std::chrono::duration_cast<std::chrono::seconds>(machine_.current_backoff()).count() << '\n';
  } else if (machine_.state() == LauncherState::Locked) {
    std::cout << "lock_reason=too_many_failures\n";
  }
}

void Launcher::logStateIfChanged(LauncherState before) const {
  if (before != machine_.state()) {
    std::cout << "launcher_state=" << launcherStateName(machine_.state()) << '\n';
  }
}

void Launcher::logLowStateIfChanged(bool connected) {
  if (!reported_low_state_initialized_ || connected != reported_low_state_connected_) {
    std::cout << "low_state_connected=" << (connected ? 1 : 0) << '\n';
    reported_low_state_initialized_ = true;
    reported_low_state_connected_ = connected;
  }
}
