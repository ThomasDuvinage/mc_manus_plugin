/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rtc/Configuration.h>
#include "ManusDevice.h"

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/videoio.hpp>

#ifdef WITH_ROS
#  include <cv_bridge/cv_bridge.h>
#  include <rclcpp/executor.hpp>
#  include <rclcpp/rclcpp.hpp>
#endif

namespace mc_plugin
{

struct ManusPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ManusPlugin() override;

private:
#ifdef WITH_ROS
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::Node::SharedPtr node_;
  std::thread ros_spin_thread_;
#endif

  std::thread capture_thread_;
  bool stop_capture_;
  std::vector<std::unique_ptr<mc_rbdyn::ManusDevice>> manuss_;
  mc_rtc::Configuration config_;
};

} // namespace mc_plugin
