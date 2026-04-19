#include "policy/TorchPolicyRunner.h"

#include <stdexcept>

TorchPolicyRunner::TorchPolicyRunner(const std::string& model_path) {
  module_ = torch::jit::load(model_path, torch::kCPU);
  module_.eval();
}

std::vector<float> TorchPolicyRunner::run(const std::vector<float>& obs) {
  if (obs.size() != 96) {
    throw std::runtime_error("Expected obs size 96, got " + std::to_string(obs.size()));
  }

  torch::NoGradGuard no_grad;
  torch::Tensor input = torch::from_blob(
      const_cast<float*>(obs.data()), {1, static_cast<long>(obs.size())}, torch::TensorOptions().dtype(torch::kFloat32))
                            .clone();
  torch::Tensor output = module_.forward({input}).toTensor().to(torch::kCPU, torch::kFloat32).contiguous().view({-1});

  const float* data = output.data_ptr<float>();
  return std::vector<float>(data, data + output.numel());
}
