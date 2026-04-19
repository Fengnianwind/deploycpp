#pragma once

#include <array>
#include <string>
#include <vector>

#include "states/FSMState.h"

class FixedPoseState : public FSMState {
 public:
  explicit FixedPoseState(const std::string& yaml_path);

  FSMStateName name() const override;
  void enter(const StateAndCmd& state, PolicyOutput& output) override;
  void run(const StateAndCmd& state, PolicyOutput& output) override;
  void exit(const StateAndCmd& state, PolicyOutput& output) override;
  FSMStateName checkChange(const StateAndCmd& state) const override;

  bool complete() const;
  int currentStep() const;
  int totalSteps() const;

 private:
  void writeTargets(PolicyOutput& output, float alpha) const;

  std::array<float, 29> init_dof_pos_{};
  std::vector<float> kps_;
  std::vector<float> kds_;
  std::vector<float> default_angles_;
  std::vector<int> joint2motor_idx_;
  double control_dt_ = 0.02;
  int cur_step_ = 0;
  int num_step_ = 100;
};
