#pragma once

#include <algorithm>
#include <array>
#include <cmath>

struct PolicyOutput {
  std::array<float, 29> actions{};
  std::array<float, 29> kps{};
  std::array<float, 29> kds{};

  void setDamping(float kd) {
    actions.fill(0.0f);
    kps.fill(0.0f);
    kds.fill(kd);
  }

  bool isFinite() const {
    const auto finite = [](float value) { return std::isfinite(value); };
    return std::all_of(actions.begin(), actions.end(), finite) &&
           std::all_of(kps.begin(), kps.end(), finite) &&
           std::all_of(kds.begin(), kds.end(), finite);
  }
};
