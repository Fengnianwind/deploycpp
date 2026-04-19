#pragma once

#include <mutex>
#include <string>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "config/Config.h"
#include "types/RemoteState.h"
#include "types/PolicyOutput.h"
#include "types/StateAndCmd.h"

class RobotIO {
 public:
  explicit RobotIO(const Config& config);

  bool hasLowState() const;
  StateAndCmd latestStateAndCmd() const;
  void sendCommand(const PolicyOutput& output);

 private:
  void lowStateHandler(const void* message);

  mutable std::mutex mutex_;
  unitree_hg::msg::dds_::LowState_ low_state_;
  unitree_hg::msg::dds_::LowCmd_ low_cmd_;
  bool has_low_state_ = false;
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
};
