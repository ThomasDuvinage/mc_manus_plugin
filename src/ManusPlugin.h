/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rtc/Configuration.h>
#include "ManusDevice.h"

#include <rclcpp/rclcpp.hpp>

namespace mc_plugin
{

struct ManusPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;
  void reset(mc_control::MCGlobalController & controller) override;
  void before(mc_control::MCGlobalController & controller) override;
  void after(mc_control::MCGlobalController & controller) override;
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;
  ~ManusPlugin() override;

private:
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::Node::SharedPtr node_;
  std::thread rosSpinThread_;

  mc_rtc::Configuration config_;
};

} // namespace mc_plugin
