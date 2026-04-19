#pragma once

#include <string>
#include <vector>

#include "config/Config.h"
#include "policy/TorchPolicyRunner.h"
#include "states/FSMState.h"

class LocoModeState : public FSMState {
 public:
  explicit LocoModeState(const Config& config);

  FSMStateName name() const override;
  void enter(const StateAndCmd& state, PolicyOutput& output) override;
  void run(const StateAndCmd& state, PolicyOutput& output) override;
  void exit(const StateAndCmd& state, PolicyOutput& output) override;
  FSMStateName checkChange(const StateAndCmd& state) const override;

#ifdef LOCO_MODE_STATE_TEST_ONLY
  std::vector<float> buildObsForTest(const StateAndCmd& state) const;
#endif

 private:
  void loadYaml(const std::string& yaml_path);
  std::vector<float> buildObs(const StateAndCmd& state) const;
  void writeOutput(const std::vector<float>& action, PolicyOutput& output) const;

  TorchPolicyRunner runner_;
  std::vector<float> prev_action_;
  std::vector<float> kps_;
  std::vector<float> kds_;
  std::vector<float> default_angles_;
  std::vector<int> joint2motor_idx_;
  std::vector<float> cmd_scale_;
  std::vector<float> cmd_min_;
  std::vector<float> cmd_max_;
  float ang_vel_scale_ = 1.0f;
  float dof_pos_scale_ = 1.0f;
  float dof_vel_scale_ = 1.0f;
  float action_scale_ = 1.0f;
  int num_obs_ = 96;
  int num_actions_ = 29;
};
