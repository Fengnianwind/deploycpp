#include <chrono>
#include <iostream>

#include "launcher/LauncherStateMachine.h"

namespace {

using Clock = std::chrono::steady_clock;

RemoteState armRemote() {
  RemoteState remote;
  remote.L2 = true;
  remote.R2 = true;
  return remote;
}

RemoteState startRemote() {
  RemoteState remote;
  remote.X = true;
  return remote;
}

}  // namespace

int main() {
  LauncherPolicy policy;
  policy.arm_timeout = std::chrono::seconds(5);
  policy.failure_window = std::chrono::seconds(60);
  policy.fast_failure_threshold = std::chrono::seconds(2);
  policy.backoff_schedule = {
      std::chrono::seconds(1),
      std::chrono::seconds(3),
      std::chrono::seconds(10),
  };

  LauncherStateMachine machine(policy);
  const auto t0 = Clock::time_point{};

  LauncherStepResult step = machine.tick(RemoteState{}, false, t0);
  if (machine.state() != LauncherState::Idle || step.start_runner) {
    std::cerr << "expected initial idle state\n";
    return 1;
  }

  step = machine.tick(armRemote(), true, t0 + std::chrono::milliseconds(10));
  if (machine.state() != LauncherState::Armed || step.start_runner) {
    std::cerr << "expected arm combo to move launcher into Armed\n";
    return 1;
  }

  step = machine.tick(startRemote(), true, t0 + std::chrono::milliseconds(20));
  if (machine.state() != LauncherState::Starting || !step.start_runner) {
    std::cerr << "expected X rising edge to trigger runner start\n";
    return 1;
  }

  machine.onRunnerSpawned(1234);
  if (machine.state() != LauncherState::Running) {
    std::cerr << "expected spawned runner to move launcher into Running\n";
    return 1;
  }

  machine.onRunnerExit(1, std::chrono::seconds(1), true, t0 + std::chrono::seconds(1));
  if (machine.state() != LauncherState::Backoff || machine.restart_attempt() != 1 ||
      machine.current_backoff() != std::chrono::seconds(1)) {
    std::cerr << "expected fast failure to enter first backoff slot\n";
    return 1;
  }

  step = machine.tick(RemoteState{}, true, t0 + std::chrono::milliseconds(2001));
  if (machine.state() != LauncherState::Starting || !step.start_runner) {
    std::cerr << "expected backoff expiry to restart runner\n";
    return 1;
  }

  machine.onRunnerSpawned(1235);
  machine.onRunnerExit(2, std::chrono::seconds(1), true, t0 + std::chrono::seconds(3));
  if (machine.state() != LauncherState::Backoff || machine.restart_attempt() != 2 ||
      machine.current_backoff() != std::chrono::seconds(3)) {
    std::cerr << "expected second failure to enter second backoff slot\n";
    return 1;
  }

  step = machine.tick(RemoteState{}, true, t0 + std::chrono::seconds(6));
  if (machine.state() != LauncherState::Starting || !step.start_runner) {
    std::cerr << "expected second backoff expiry to restart runner\n";
    return 1;
  }

  machine.onRunnerSpawned(1236);
  machine.onRunnerExit(3, std::chrono::seconds(1), true, t0 + std::chrono::seconds(7));
  if (machine.state() != LauncherState::Backoff || machine.restart_attempt() != 3 ||
      machine.current_backoff() != std::chrono::seconds(10)) {
    std::cerr << "expected third failure to enter third backoff slot\n";
    return 1;
  }

  step = machine.tick(RemoteState{}, true, t0 + std::chrono::seconds(18));
  if (machine.state() != LauncherState::Starting || !step.start_runner) {
    std::cerr << "expected third backoff expiry to allow final restart\n";
    return 1;
  }

  machine.onRunnerSpawned(1237);
  machine.onRunnerExit(4, std::chrono::seconds(1), true, t0 + std::chrono::seconds(19));
  if (machine.state() != LauncherState::Locked || machine.restart_attempt() != 4) {
    std::cerr << "expected fourth failure to lock launcher\n";
    return 1;
  }

  step = machine.tick(RemoteState{}, true, t0 + std::chrono::seconds(20));
  if (step.start_runner) {
    std::cerr << "locked launcher must not auto restart\n";
    return 1;
  }

  step = machine.tick(armRemote(), true, t0 + std::chrono::seconds(21));
  if (machine.state() != LauncherState::Armed || machine.restart_attempt() != 0) {
    std::cerr << "expected re-arm after lock to clear restart budget\n";
    return 1;
  }

  LauncherStateMachine timeout_machine(policy);
  timeout_machine.tick(armRemote(), true, t0 + std::chrono::milliseconds(10));
  timeout_machine.tick(RemoteState{}, true, t0 + std::chrono::seconds(6));
  if (timeout_machine.state() != LauncherState::Idle) {
    std::cerr << "expected Armed timeout to return to Idle\n";
    return 1;
  }

  std::cout << "launcher_state_machine_test_pass=1\n";
  return 0;
}
