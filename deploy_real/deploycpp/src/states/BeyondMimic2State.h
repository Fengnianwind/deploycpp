#pragma once

#include <array>
#include <string>
#include <vector>

#include "config/Config.h"
#include "math/QuaternionUtils.h"
#include "policy/OnnxPolicyRunner.h"
#include "states/FSMState.h"

class BeyondMimic2State : public FSMState {
 public:
  explicit BeyondMimic2State(const Config& config);

  FSMStateName name() const override;
  void enter(const StateAndCmd& state, PolicyOutput& output) override;
  void run(const StateAndCmd& state, PolicyOutput& output) override;
  void exit(const StateAndCmd& state, PolicyOutput& output) override;
  FSMStateName checkChange(const StateAndCmd& state) const override;

 private:
  void loadYaml(const std::string& yaml_path);
  Quat torsoAdjustedQuat(const std::array<float, 29>& q, const Quat& base_quat, std::vector<float>& qj_mj2lab) const;
  std::vector<float> buildMimicObs(
      const std::vector<float>& qj_mj2lab,
      const std::array<float, 29>& dq,
      const std::array<float, 3>& ang_vel,
      const Mat3& motion_anchor_ori_b) const;
  Mat3 computeMotionAnchor(const Quat& robot_quat) const;
  void writeHoldOutput(const StateAndCmd& state, PolicyOutput& output) const;
  void writePolicyOutput(const PolicyRunResult& result, PolicyOutput& output) const;

  OnnxPolicyRunner runner_;
  std::vector<int> mj2lab_;
  std::vector<float> kps_lab_;
  std::vector<float> kds_lab_;
  std::vector<float> default_angles_lab_;
  std::vector<float> action_scale_lab_;
  int num_obs_ = 154;
  int counter_step_ = 0;
  PolicyRunResult current_result_;
  Mat3 init_to_world_{};
};
