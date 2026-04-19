#include "states/BeyondMimic2State.h"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
std::vector<T> requiredVector(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing BeyondMimic2 key: " + key);
  }
  return node[key].as<std::vector<T>>();
}

float matrixAt(const Mat3& m, int row, int col) {
  return m[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
}

Mat3 transpose(const Mat3& m) {
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = matrixAt(m, c, r);
    }
  }
  return out;
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float value = 0.0f;
      for (int k = 0; k < 3; ++k) {
        value += matrixAt(a, r, k) * matrixAt(b, k, c);
      }
      out[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = value;
    }
  }
  return out;
}

std::vector<float> flattenFirstTwoColumns(const Mat3& m) {
  std::vector<float> out;
  out.reserve(6);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 2; ++c) {
      out.push_back(matrixAt(m, r, c));
    }
  }
  return out;
}

}  // namespace

BeyondMimic2State::BeyondMimic2State(const Config& config)
    : runner_(config.beyond_mimic2_model_path) {
  loadYaml(config.beyond_mimic2_yaml_path);
}

FSMStateName BeyondMimic2State::name() const {
  return FSMStateName::BeyondMimic2;
}

void BeyondMimic2State::enter(const StateAndCmd& state, PolicyOutput& output) {
  counter_step_ = 0;
  std::vector<float> zero_obs(static_cast<std::size_t>(num_obs_), 0.0f);
  current_result_ = runner_.run(zero_obs, 0.0f);
  writeHoldOutput(state, output);
}

void BeyondMimic2State::run(const StateAndCmd& state, PolicyOutput& output) {
  std::vector<float> qj_mj2lab;
  const Quat robot_quat = torsoAdjustedQuat(state.q, state.base_quat, qj_mj2lab);

  if (counter_step_ < 2) {
    init_to_world_ = computeMotionAnchor(robot_quat);
    ++counter_step_;
    writeHoldOutput(state, output);
    return;
  }

  const Quat ref_anchor_ori_w{
      current_result_.body_quat_w[7 * 4 + 0],
      current_result_.body_quat_w[7 * 4 + 1],
      current_result_.body_quat_w[7 * 4 + 2],
      current_result_.body_quat_w[7 * 4 + 3],
  };
  const Mat3 motion_anchor_ori_b =
      multiply(multiply(transpose(matrixFromQuat(robot_quat)), init_to_world_), matrixFromQuat(ref_anchor_ori_w));
  const std::vector<float> mimic_obs = buildMimicObs(qj_mj2lab, state.dq, state.ang_vel, motion_anchor_ori_b);
  current_result_ = runner_.run(mimic_obs, static_cast<float>(counter_step_));
  writePolicyOutput(current_result_, output);
  ++counter_step_;
}

void BeyondMimic2State::exit(const StateAndCmd&, PolicyOutput&) {
  counter_step_ = 0;
}

FSMStateName BeyondMimic2State::checkChange(const StateAndCmd& state) const {
  if (state.remote.F1) {
    return FSMStateName::Passive;
  }
  if (state.remote.start) {
    return FSMStateName::FixedPose;
  }
  if (state.remote.R1 && state.remote.A) {
    return FSMStateName::LocoMode;
  }
  return FSMStateName::BeyondMimic2;
}

void BeyondMimic2State::loadYaml(const std::string& yaml_path) {
  const YAML::Node root = YAML::LoadFile(yaml_path);
  mj2lab_ = requiredVector<int>(root, "mj2lab");
  kps_lab_ = requiredVector<float>(root, "kp_lab");
  kds_lab_ = requiredVector<float>(root, "kd_lab");
  default_angles_lab_ = requiredVector<float>(root, "default_angles_lab");
  action_scale_lab_ = requiredVector<float>(root, "action_scale_lab");
  if (!root["num_obs"]) {
    throw std::runtime_error("Missing BeyondMimic2 key: num_obs");
  }
  num_obs_ = root["num_obs"].as<int>();

  if (mj2lab_.size() != 29 || kps_lab_.size() != 29 || kds_lab_.size() != 29 ||
      default_angles_lab_.size() != 29 || action_scale_lab_.size() != 29) {
    throw std::runtime_error("BeyondMimic2 config arrays must all have size 29");
  }
}

Quat BeyondMimic2State::torsoAdjustedQuat(
    const std::array<float, 29>& q, const Quat& base_quat, std::vector<float>& qj_mj2lab) const {
  qj_mj2lab.clear();
  qj_mj2lab.reserve(29);
  for (std::size_t i = 0; i < mj2lab_.size(); ++i) {
    qj_mj2lab.push_back(q[static_cast<std::size_t>(mj2lab_[i])] - default_angles_lab_[i]);
  }

  const float base_torso_yaw = qj_mj2lab[2];
  const float base_torso_roll = qj_mj2lab[5];
  const float base_torso_pitch = qj_mj2lab[8];

  const Quat quat_yaw = eulerSingleAxisToQuat(base_torso_yaw, 'z');
  const Quat quat_roll = eulerSingleAxisToQuat(base_torso_roll, 'x');
  const Quat quat_pitch = eulerSingleAxisToQuat(base_torso_pitch, 'y');
  return quatMul(base_quat, quatMul(quat_yaw, quatMul(quat_roll, quat_pitch)));
}

std::vector<float> BeyondMimic2State::buildMimicObs(
    const std::vector<float>& qj_mj2lab,
    const std::array<float, 29>& dq,
    const std::array<float, 3>& ang_vel,
    const Mat3& motion_anchor_ori_b) const {
  std::vector<float> mimic_obs;
  mimic_obs.reserve(static_cast<std::size_t>(num_obs_));
  mimic_obs.insert(mimic_obs.end(), current_result_.joint_pos.begin(), current_result_.joint_pos.end());
  mimic_obs.insert(mimic_obs.end(), current_result_.joint_vel.begin(), current_result_.joint_vel.end());
  const std::vector<float> motion_first_two_cols = flattenFirstTwoColumns(motion_anchor_ori_b);
  mimic_obs.insert(mimic_obs.end(), motion_first_two_cols.begin(), motion_first_two_cols.end());
  mimic_obs.insert(mimic_obs.end(), ang_vel.begin(), ang_vel.end());
  mimic_obs.insert(mimic_obs.end(), qj_mj2lab.begin(), qj_mj2lab.end());
  for (const int index : mj2lab_) {
    mimic_obs.push_back(dq[static_cast<std::size_t>(index)]);
  }
  mimic_obs.insert(mimic_obs.end(), current_result_.actions.begin(), current_result_.actions.end());

  if (mimic_obs.size() != static_cast<std::size_t>(num_obs_)) {
    throw std::runtime_error("Expected mimic_obs size " + std::to_string(num_obs_) + ", got " +
                             std::to_string(mimic_obs.size()));
  }
  return mimic_obs;
}

Mat3 BeyondMimic2State::computeMotionAnchor(const Quat& robot_quat) const {
  const Quat ref_anchor_ori_w{
      current_result_.body_quat_w[7 * 4 + 0],
      current_result_.body_quat_w[7 * 4 + 1],
      current_result_.body_quat_w[7 * 4 + 2],
      current_result_.body_quat_w[7 * 4 + 3],
  };
  const Mat3 init_to_anchor = matrixFromQuat(yawQuat(ref_anchor_ori_w));
  const Mat3 world_to_anchor = matrixFromQuat(yawQuat(robot_quat));
  return multiply(world_to_anchor, transpose(init_to_anchor));
}

void BeyondMimic2State::writeHoldOutput(const StateAndCmd& state, PolicyOutput& output) const {
  output.actions.fill(0.0f);
  output.kps.fill(0.0f);
  output.kds.fill(0.0f);
  for (std::size_t i = 0; i < mj2lab_.size(); ++i) {
    const std::size_t motor_idx = static_cast<std::size_t>(mj2lab_[i]);
    output.actions[motor_idx] = state.q[motor_idx];
    output.kps[motor_idx] = kps_lab_[i];
    output.kds[motor_idx] = kds_lab_[i];
  }
}

void BeyondMimic2State::writePolicyOutput(const PolicyRunResult& result, PolicyOutput& output) const {
  for (std::size_t i = 0; i < mj2lab_.size(); ++i) {
    const std::size_t motor_idx = static_cast<std::size_t>(mj2lab_[i]);
    output.actions[motor_idx] = result.actions[i] * action_scale_lab_[i] + default_angles_lab_[i];
    output.kps[motor_idx] = kps_lab_[i];
    output.kds[motor_idx] = kds_lab_[i];
  }
}
