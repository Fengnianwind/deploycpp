#include "RobotIO.h"

#include <chrono>
#include <functional>
#include <stdexcept>

#include <unitree/robot/channel/channel_factory.hpp>

#include "robot/CommandUtils.h"
#include "robot/RemoteControl.h"

RobotIO::RobotIO(const Config& config) {
  unitree::robot::ChannelFactory::Instance()->Init(0, config.net);
  lowcmd_publisher_.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(config.lowcmd_topic));
  lowstate_subscriber_.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(config.lowstate_topic));
  lowcmd_publisher_->InitChannel();
  lowstate_subscriber_->InitChannel(std::bind(&RobotIO::lowStateHandler, this, std::placeholders::_1));
}

bool RobotIO::hasLowState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_low_state_;
}

StateAndCmd RobotIO::latestStateAndCmd() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_low_state_) {
    throw std::runtime_error("No low_state received");
  }

  StateAndCmd state;
  for (std::size_t i = 0; i < state.q.size(); ++i) {
    state.q[i] = low_state_.motor_state()[i].q();
    state.dq[i] = low_state_.motor_state()[i].dq();
  }
  for (std::size_t i = 0; i < state.base_quat.size(); ++i) {
    state.base_quat[i] = low_state_.imu_state().quaternion()[i];
  }
  for (std::size_t i = 0; i < state.ang_vel.size(); ++i) {
    state.ang_vel[i] = low_state_.imu_state().gyroscope()[i];
  }
  const auto& raw = low_state_.wireless_remote();
  state.remote = decodeRemoteState(raw.data(), raw.size());
  state.vel_cmd[0] = state.remote.ly;
  state.vel_cmd[1] = -state.remote.lx;
  state.vel_cmd[2] = -state.remote.rx;
  return state;
}

void RobotIO::sendCommand(const PolicyOutput& output) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_low_state_) {
    throw std::runtime_error("Cannot send command before low_state is received");
  }
  buildLowCommand(low_cmd_, low_state_, output);
  lowcmd_publisher_->Write(low_cmd_);
}

void RobotIO::lowStateHandler(const void* message) {
  const auto* low_state = static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
  std::lock_guard<std::mutex> lock(mutex_);
  low_state_ = *low_state;
  has_low_state_ = true;
}
