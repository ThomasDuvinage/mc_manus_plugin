#include "ManusPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

ManusPlugin::~ManusPlugin()
{
#ifdef WITH_ROS
  executor_->cancel();
  if(ros_spin_thread_.joinable())
  {
    ros_spin_thread_.join();
  }
#endif

  stop_capture_ = true;
  if(capture_thread_.joinable())
  {
    capture_thread_.join();
  }
};

void ManusPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  config_.load(config);
  reset(controller);
}

void ManusPlugin::after(mc_control::MCGlobalController & controller) {}

void ManusPlugin::reset(mc_control::MCGlobalController & controller)
{
  auto manusConfig = config_.find("ManusPlugin");
  manuss_.clear();
  deviceNames_.clear();

#ifdef WITH_ROS
  node_ = rclcpp::Node::make_shared("ManusPlugin");
  executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  executor_->add_node(node_);
  rosSpinThread_ = std::thread([&]() { executor_->spin(); });
#endif

  if(!manusConfig)
  {
    mc_rtc::log::error("[ManusPlugin] Cannot find ManusPlugin configuration");
    return;
  }

  for(const auto & entry : *manusConfig)
  {
    const std::string name = entry("name");
    const std::string parent = entry("parent");
    sva::PTransformd transform = entry.has("manusTransform") ? entry("manusTransform") : sva::PTransformd::Identity();

    if(entry.has("topic"))
    {
#ifdef WITH_ROS
      const std::string topic = entry("topic");
      manuss_.push_back(std::make_unique<mc_rbdyn::ManusDevice>(name, parent, transform, topic, node_));
      mc_rtc::log::info("[ManusPlugin] Subscribed {} to {}", name, topic);
#else
      mc_rtc::log::error_and_throw("[ManusPlugin] ROS support disabled: cannot subscribe to topic for {}", name);
#endif
    }
    else
    {
      mc_rtc::log::error_and_throw("[ManusPlugin] Missing \"topic\" for Manus device {}", name);
    }
  }

  for(auto & glove : manuss_)
  {
    const std::string gloveName = glove->name();
    if(!controller.robot().hasDevice<mc_rbdyn::ManusDevice>(gloveName))
    {
      controller.robot().addDevice(std::move(glove));
      controller.robot()
          .device<mc_rbdyn::ManusDevice>(controller.robot().devices().back()->name())
          .addToGUI(*controller.controller().gui());
    }
    deviceNames_.push_back(gloveName);
  }

  for(const auto & gloveName : deviceNames_)
  {
    controller.datastore().make<mc_rbdyn::ManusDevice::Data>(
        "manus/" + gloveName,
        controller.robot().device<mc_rbdyn::ManusDevice>(gloveName).data());
  }
}

void ManusPlugin::before(mc_control::MCGlobalController & controller)
{
  for(const auto & gloveName : deviceNames_)
  {
    auto & device = controller.robot().device<mc_rbdyn::ManusDevice>(gloveName);
    controller.datastore().assign("manus/" + gloveName, device.data());
  }
}


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
