#pragma once

#include <string>
#include <vector>

#include <torch/script.h>

class TorchPolicyRunner {
 public:
  explicit TorchPolicyRunner(const std::string& model_path);

  std::vector<float> run(const std::vector<float>& obs);

 private:
  torch::jit::script::Module module_;
};
