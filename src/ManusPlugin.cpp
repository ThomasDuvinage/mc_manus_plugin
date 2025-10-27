#include "ManusPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

ManusPlugin::~ManusPlugin()
{
  if(executor_)
  {
    executor_->cancel();
  }
  if(rosSpinThread_.joinable())
  {
    rosSpinThread_.join();
  }
  executor_.reset();
}

void ManusPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  config_.load(config);
  reset(controller);
}

void ManusPlugin::after(mc_control::MCGlobalController & controller) {}

void ManusPlugin::reset(mc_control::MCGlobalController & controller)
{
  auto manusConfig = config_.find("ManusPlugin");

  node_ = rclcpp::Node::make_shared("ManusPlugin");
  executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  executor_->add_node(node_);
  rosSpinThread_ = std::thread([&]() { executor_->spin(); });

  if(!manusConfig)
  {
    mc_rtc::log::error("[ManusPlugin] Cannot find ManusPlugin configuration");
    return;
  }

  for(const auto & entry : *manusConfig)
  {
    const std::string name = entry("name");

    if(entry.has("topic"))
    {
      const std::string topic = entry("topic");
      auto manus = std::make_unique<mc_rbdyn::ManusDevice>(name, topic, node_);

      if(!controller.robot().hasDevice<mc_rbdyn::ManusDevice>(name))
      {
        controller.robot().addDevice(std::move(manus));
        controller.robot()
            .device<mc_rbdyn::ManusDevice>(controller.robot().devices().back()->name())
            .addToGUI(*controller.controller().gui());
      }

      mc_rtc::log::info("[ManusPlugin] Subscribed {} to {}", name, topic);
    }
    else
    {
      mc_rtc::log::error_and_throw("[ManusPlugin] Missing \"topic\" for Manus device {}", name);
    }
  }
}

void ManusPlugin::before(mc_control::MCGlobalController & controller) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration ManusPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ManusPlugin", mc_plugin::ManusPlugin)
