#include "policy/OnnxPolicyRunner.h"

#include <array>
#include <cstring>
#include <numeric>
#include <stdexcept>

namespace {

std::vector<std::string> readInputNames(Ort::Session& session, Ort::AllocatorWithDefaultOptions& allocator) {
  std::vector<std::string> names;
  const std::size_t count = session.GetInputCount();
  names.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    Ort::AllocatedStringPtr name = session.GetInputNameAllocated(i, allocator);
    names.emplace_back(name.get());
  }
  return names;
}

std::vector<std::string> readOutputNames(Ort::Session& session, Ort::AllocatorWithDefaultOptions& allocator) {
  std::vector<std::string> names;
  const std::size_t count = session.GetOutputCount();
  names.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(i, allocator);
    names.emplace_back(name.get());
  }
  return names;
}

std::vector<const char*> rawNames(const std::vector<std::string>& names) {
  std::vector<const char*> raw;
  raw.reserve(names.size());
  for (const auto& name : names) {
    raw.push_back(name.c_str());
  }
  return raw;
}

void expectName(const std::vector<std::string>& names, std::size_t index, const std::string& expected) {
  if (index >= names.size() || names[index] != expected) {
    throw std::runtime_error("Unexpected ONNX name at index " + std::to_string(index) + ": expected " + expected);
  }
}

}  // namespace

OnnxPolicyRunner::OnnxPolicyRunner(const std::string& model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "robomimic_deploycpp"),
      session_options_{},
      session_(nullptr) {
  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
  session_ = Ort::Session(env_, model_path.c_str(), session_options_);
  input_names_ = readInputNames(session_, allocator_);
  output_names_ = readOutputNames(session_, allocator_);
  validateContract();
}

std::vector<std::string> OnnxPolicyRunner::inputNames() const {
  return input_names_;
}

std::vector<std::string> OnnxPolicyRunner::outputNames() const {
  return output_names_;
}

PolicyRunResult OnnxPolicyRunner::run(const std::vector<float>& obs, float time_step) {
  if (obs.size() != 154) {
    throw std::runtime_error("Expected obs size 154, got " + std::to_string(obs.size()));
  }

  std::array<int64_t, 2> obs_shape{1, 154};
  std::array<int64_t, 2> time_shape{1, 1};
  std::array<float, 1> time_data{time_step};

  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<Ort::Value> inputs;
  inputs.emplace_back(Ort::Value::CreateTensor<float>(
      memory_info, const_cast<float*>(obs.data()), obs.size(), obs_shape.data(), obs_shape.size()));
  inputs.emplace_back(Ort::Value::CreateTensor<float>(
      memory_info, time_data.data(), time_data.size(), time_shape.data(), time_shape.size()));

  const std::vector<const char*> input_name_ptrs = rawNames(input_names_);
  const std::vector<const char*> output_name_ptrs = rawNames(output_names_);
  std::vector<Ort::Value> outputs = session_.Run(
      Ort::RunOptions{nullptr},
      input_name_ptrs.data(),
      inputs.data(),
      inputs.size(),
      output_name_ptrs.data(),
      output_name_ptrs.size());

  if (outputs.size() != 7) {
    throw std::runtime_error("Expected 7 ONNX outputs, got " + std::to_string(outputs.size()));
  }

  PolicyRunResult result;
  result.actions = outputToVector(outputs[0], "actions", 29);
  result.joint_pos = outputToVector(outputs[1], "joint_pos", 29);
  result.joint_vel = outputToVector(outputs[2], "joint_vel", 29);
  result.body_pos_w = outputToVector(outputs[3], "body_pos_w", 42);
  result.body_quat_w = outputToVector(outputs[4], "body_quat_w", 56);
  result.body_lin_vel_w = outputToVector(outputs[5], "body_lin_vel_w", 42);
  result.body_ang_vel_w = outputToVector(outputs[6], "body_ang_vel_w", 42);
  return result;
}

std::vector<float> OnnxPolicyRunner::outputToVector(
    Ort::Value& value, const std::string& name, std::size_t expected_size) const {
  Ort::TensorTypeAndShapeInfo info = value.GetTensorTypeAndShapeInfo();
  const std::vector<int64_t> shape = info.GetShape();
  const std::size_t element_count = static_cast<std::size_t>(info.GetElementCount());
  if (element_count != expected_size) {
    throw std::runtime_error(
        "Unexpected element count for " + name + ": expected " + std::to_string(expected_size) +
        ", got " + std::to_string(element_count));
  }
  const float* data = value.GetTensorData<float>();
  return std::vector<float>(data, data + element_count);
}

void OnnxPolicyRunner::validateContract() const {
  if (input_names_.size() != 2) {
    throw std::runtime_error("Expected 2 ONNX inputs, got " + std::to_string(input_names_.size()));
  }
  if (output_names_.size() != 7) {
    throw std::runtime_error("Expected 7 ONNX outputs, got " + std::to_string(output_names_.size()));
  }

  expectName(input_names_, 0, "obs");
  expectName(input_names_, 1, "time_step");
  expectName(output_names_, 0, "actions");
  expectName(output_names_, 1, "joint_pos");
  expectName(output_names_, 2, "joint_vel");
  expectName(output_names_, 3, "body_pos_w");
  expectName(output_names_, 4, "body_quat_w");
  expectName(output_names_, 5, "body_lin_vel_w");
  expectName(output_names_, 6, "body_ang_vel_w");
}
