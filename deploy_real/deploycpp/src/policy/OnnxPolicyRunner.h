#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

struct PolicyRunResult {
  std::vector<float> actions;
  std::vector<float> joint_pos;
  std::vector<float> joint_vel;
  std::vector<float> body_pos_w;
  std::vector<float> body_quat_w;
  std::vector<float> body_lin_vel_w;
  std::vector<float> body_ang_vel_w;
};

class OnnxPolicyRunner {
 public:
  explicit OnnxPolicyRunner(const std::string& model_path);

  std::vector<std::string> inputNames() const;
  std::vector<std::string> outputNames() const;
  PolicyRunResult run(const std::vector<float>& obs, float time_step);

 private:
  std::vector<float> outputToVector(Ort::Value& value, const std::string& name, std::size_t expected_size) const;
  void validateContract() const;

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  Ort::AllocatorWithDefaultOptions allocator_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
};
