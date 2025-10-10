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
  const auto & manus_plugin_config_ = config_.find("ManusPlugin");
  stop_capture_ = false;
  manuss_.clear();

  std::cout << manus_plugin_config_->dump(true, true) << std::endl;

#ifdef WITH_ROS
  node_ = rclcpp::Node::make_shared("ManusPlugin");
  executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  executor_->add_node(node_);

  ros_spin_thread_ = std::thread([&]() { executor_->spin(); });
#endif

  if(manus_plugin_config_)
  {
    for(auto it = manus_plugin_config_->begin(); it != manus_plugin_config_->end(); ++it)
    {
      const auto & cam_cfg = *it;

      std::string name = cam_cfg("name");
      std::string parent = cam_cfg("parent");
      sva::PTransformd manusTransform = sva::PTransformd::Identity();

      if(cam_cfg.has("manusTransform"))
      {
        manusTransform = cam_cfg("manusTransform");
      }

      if(cam_cfg.has("index") || cam_cfg.has("hand_topic"))
      {
        if(cam_cfg.has("index") && cam_cfg("index").isInteger())
        {
          manuss_.push_back(std::make_unique<mc_rbdyn::ManusDevice>(name, parent, manusTransform,
                                                                      static_cast<int>(cam_cfg("index"))));
          mc_rtc::log::info("[ManusPlugin] Manus {} Initialized", name);
        }
        else if(cam_cfg("hand_topic").isString())
        {
#ifdef WITH_ROS
          if(cam_cfg.has("compressed"))
          {
            manuss_.push_back(std::make_unique<mc_rbdyn::ManusDevice>(
                name, parent, manusTransform, static_cast<std::string>(cam_cfg("hand_topic")), cam_cfg("compressed"),
                node_));
          }
          else
          {
            manuss_.push_back(std::make_unique<mc_rbdyn::ManusDevice>(
                name, parent, manusTransform, static_cast<std::string>(cam_cfg("hand_topic")), false, node_));
          }
#else
          mc_rtc::log::error_and_throw(
              "[ManusPlugin] Please compile with WITH_ROS option to enable ROS functionnalities");
#endif
          mc_rtc::log::info("[ManusPlugin] Manus {} Initialized", name);
        }
        else
        {
          mc_rtc::log::error_and_throw("[ManusPlugin] index (int) or hand_topic (string)");
        }
      }
      else
      {
        mc_rtc::log::error_and_throw("[ManusPlugin] An index or hand_topic should be given for manus : {}", name);
      }
    }

    for(auto & c : manuss_)
    {
      if(!controller.robot().hasDevice<mc_rbdyn::ManusDevice>(c->name()))
      {
        controller.robot().addDevice(std::move(c));
        controller.robot()
            .device<mc_rbdyn::ManusDevice>(controller.robot().devices().back()->name())
            .addToGUI(*controller.controller().gui());
      }
    }
  }
  else
  {
    mc_rtc::log::error("[ManusPlugin] Cannot find ManusPlugin configuration");
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
