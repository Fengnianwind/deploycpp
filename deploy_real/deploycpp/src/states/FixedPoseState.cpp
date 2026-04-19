#include "states/FixedPoseState.h"

#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
std::vector<T> requiredVector(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing FixedPose key: " + key);
  }
  return node[key].as<std::vector<T>>();
}

}  // namespace

FixedPoseState::FixedPoseState(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  kps_ = requiredVector<float>(root, "kps");
  kds_ = requiredVector<float>(root, "kds");
  default_angles_ = requiredVector<float>(root, "default_angles");
  joint2motor_idx_ = requiredVector<int>(root, "joint2motor_idx");
  if (!root["control_dt"]) {
    throw std::runtime_error("Missing FixedPose key: control_dt");
  }
  control_dt_ = root["control_dt"].as<double>();
  num_step_ = static_cast<int>(2.0 / control_dt_);

  if (kps_.size() != 29 || kds_.size() != 29 || default_angles_.size() != 29 || joint2motor_idx_.size() != 29) {
    throw std::runtime_error("FixedPose config arrays must all have size 29");
  }
}

FSMStateName FixedPoseState::name() const {
  return FSMStateName::FixedPose;
}

void FixedPoseState::enter(const StateAndCmd& state, PolicyOutput& output) {
  init_dof_pos_ = state.q;
  cur_step_ = 0;
  writeTargets(output, 0.0f);
}

void FixedPoseState::run(const StateAndCmd&, PolicyOutput& output) {
  cur_step_ += 1;
  const float alpha = std::min(static_cast<float>(cur_step_) / static_cast<float>(num_step_), 1.0f);
  writeTargets(output, alpha);
}

void FixedPoseState::exit(const StateAndCmd&, PolicyOutput& output) {
  writeTargets(output, 1.0f);
}

FSMStateName FixedPoseState::checkChange(const StateAndCmd& state) const {
  if (state.remote.F1) {
    return FSMStateName::Passive;
  }
  if (state.remote.R1 && state.remote.A && complete()) {
    return FSMStateName::LocoMode;
  }
  if (state.remote.L1 && state.remote.Y && complete()) {
    return FSMStateName::BeyondMimic;
  }
  if (state.remote.R1 && state.remote.B && complete()) {
    return FSMStateName::BeyondMimic2;
  }
  return FSMStateName::FixedPose;
}

bool FixedPoseState::complete() const {
  return cur_step_ >= num_step_;
}

int FixedPoseState::currentStep() const {
  return cur_step_;
}

int FixedPoseState::totalSteps() const {
  return num_step_;
}

void FixedPoseState::writeTargets(PolicyOutput& output, float alpha) const {
  for (std::size_t j = 0; j < joint2motor_idx_.size(); ++j) {
    const int motor_idx = joint2motor_idx_[j];
    if (motor_idx < 0 || motor_idx >= static_cast<int>(output.actions.size())) {
      throw std::runtime_error("FixedPose joint2motor_idx out of range");
    }
    output.actions[motor_idx] = init_dof_pos_[motor_idx] * (1.0f - alpha) + default_angles_[j] * alpha;
    output.kps[motor_idx] = kps_[j];
    output.kds[motor_idx] = kds_[j];
  }
}
