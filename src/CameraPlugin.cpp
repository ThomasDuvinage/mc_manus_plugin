#include "CameraPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CameraPlugin::~CameraPlugin()
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

void CameraPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  config_.load(config);
  reset(controller);
}

void CameraPlugin::after(mc_control::MCGlobalController & controller) {}

void CameraPlugin::reset(mc_control::MCGlobalController & controller)
{
  const auto & camera_plugin_config_ = config_.find("CameraPlugin");
  stop_capture_ = false;
  cameras_.clear();

  std::cout << camera_plugin_config_->dump(true, true) << std::endl;

#ifdef WITH_ROS
  node_ = rclcpp::Node::make_shared("CameraPlugin");
  executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  executor_->add_node(node_);

  ros_spin_thread_ = std::thread([&]() { executor_->spin(); });
#endif

  if(camera_plugin_config_)
  {
    for(auto it = camera_plugin_config_->begin(); it != camera_plugin_config_->end(); ++it)
    {
      const auto & cam_cfg = *it;

      std::string name = cam_cfg("name");
      std::string parent = cam_cfg("parent");
      sva::PTransformd cameraTransform = sva::PTransformd::Identity();

      if(cam_cfg.has("cameraTransform"))
      {
        cameraTransform = cam_cfg("cameraTransform");
      }

      if(cam_cfg.has("index") || cam_cfg.has("image_topic"))
      {
        if(cam_cfg.has("index") && cam_cfg("index").isInteger())
        {
          cameras_.push_back(std::make_unique<mc_rbdyn::CameraDevice>(name, parent, cameraTransform,
                                                                      static_cast<int>(cam_cfg("index"))));
          mc_rtc::log::info("[CameraPlugin] Camera {} Initialized", name);
        }
        else if(cam_cfg("image_topic").isString())
        {
#ifdef WITH_ROS
          if(cam_cfg.has("compressed"))
          {
            cameras_.push_back(std::make_unique<mc_rbdyn::CameraDevice>(
                name, parent, cameraTransform, static_cast<std::string>(cam_cfg("image_topic")), cam_cfg("compressed"),
                node_));
          }
          else
          {
            cameras_.push_back(std::make_unique<mc_rbdyn::CameraDevice>(
                name, parent, cameraTransform, static_cast<std::string>(cam_cfg("image_topic")), false, node_));
          }
#else
          mc_rtc::log::error_and_throw(
              "[CameraPlugin] Please compile with WITH_ROS option to enable ROS functionnalities");
#endif
          mc_rtc::log::info("[CameraPlugin] Camera {} Initialized", name);
        }
        else
        {
          mc_rtc::log::error_and_throw("[CameraPlugin] index (int) or image_topic (string)");
        }
      }
      else
      {
        mc_rtc::log::error_and_throw("[CameraPlugin] An index or image_topic should be given for camera : {}", name);
      }
    }

    for(auto & c : cameras_)
    {
      if(!controller.robot().hasDevice<mc_rbdyn::CameraDevice>(c->name()))
      {
        controller.robot().addDevice(std::move(c));
        controller.robot()
            .device<mc_rbdyn::CameraDevice>(controller.robot().devices().back()->name())
            .addToGUI(*controller.controller().gui());
      }
    }
  }
  else
  {
    mc_rtc::log::error("[CameraPlugin] Cannot find CameraPlugin configuration");
  }
}

void CameraPlugin::before(mc_control::MCGlobalController & controller) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration CameraPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CameraPlugin", mc_plugin::CameraPlugin)
