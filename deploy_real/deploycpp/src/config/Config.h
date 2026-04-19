#pragma once

#include <string>

struct Config {
  std::string net;
  int num_joints = 29;
  std::string lowcmd_topic = "rt/lowcmd";
  std::string lowstate_topic = "rt/lowstate";
  double control_dt = 0.02;
  int error_over_time = 5;
  std::string project_root;
  std::string real_yaml_path;
  std::string fixed_pose_yaml_path;
  std::string passive_yaml_path;
  std::string loco_mode_yaml_path;
  std::string loco_mode_model_path;
  std::string beyond_mimic2_yaml_path;
  std::string beyond_mimic2_model_path;
  std::string beyond_mimic_yaml_path;
  std::string beyond_mimic_model_path;
};

Config loadConfig(const std::string& project_root);
