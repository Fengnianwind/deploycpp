#pragma once

#include "FSMController.h"
#include "RobotIO.h"
#include "config/Config.h"

class Controller {
 public:
  explicit Controller(const Config& config);

  int run();

 private:
  void sendDamping();

  Config config_;
  RobotIO robot_io_;
  FSMController fsm_;
  PolicyOutput output_;
  bool fixed_pose_complete_reported_ = false;
};
