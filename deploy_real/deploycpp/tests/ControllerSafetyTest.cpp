#include <iostream>
#include <stdexcept>

#include "ControllerSafety.h"

int main() {
  bool work_called = false;
  bool fallback_called = false;

  try {
    runWithDampingFallback(
        [&]() {
          work_called = true;
          throw std::runtime_error("control step failed");
        },
        [&]() { fallback_called = true; });
    std::cerr << "expected control step exception\n";
    return 1;
  } catch (const std::runtime_error&) {
  }

  if (!work_called) {
    std::cerr << "expected work callback to run\n";
    return 1;
  }
  if (!fallback_called) {
    std::cerr << "expected fallback callback to run before rethrow\n";
    return 1;
  }

  fallback_called = false;
  try {
    runWithDampingFallback(
        [&]() { throw std::runtime_error("control step failed"); },
        [&]() {
          fallback_called = true;
          throw std::runtime_error("fallback failed");
        });
    std::cerr << "expected original exception to be rethrown\n";
    return 1;
  } catch (const std::runtime_error& exc) {
    if (std::string(exc.what()) != "control step failed") {
      std::cerr << "expected original exception to win\n";
      return 1;
    }
  }

  if (!fallback_called) {
    std::cerr << "expected fallback callback even if it throws\n";
    return 1;
  }

  std::cout << "controller_safety_test_pass=1\n";
  return 0;
}
