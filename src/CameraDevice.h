#pragma once

#include <mc_rbdyn/Device.h>
#include <mc_rtc/gui.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/videoio.hpp>
#include <thread>

#ifdef WITH_ROS
#  include <cv_bridge/cv_bridge.h>
#  include <image_transport/image_transport.hpp>
#  include <image_transport/subscriber.hpp>
#  include <rclcpp/executor.hpp>
#  include <rclcpp/rclcpp.hpp>
#endif

namespace mc_rbdyn
{
struct MC_RBDYN_DLLAPI CameraDevice : public Device
{
  CameraDevice();
  ~CameraDevice() noexcept override;

  CameraDevice(const CameraDevice & cd);

  CameraDevice & operator=(const CameraDevice & fs);

  CameraDevice(CameraDevice &&) noexcept = default;
  CameraDevice & operator=(CameraDevice &&) = default;

  CameraDevice(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f);
  CameraDevice(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f, int id);

  const cv::Mat & getImage();
  void setImage(const cv::Mat & image);

  /** Return the sensor's parent body */
  inline const std::string & parentBody() const
  {
    return Device::parent();
  }

  /** Return the transformation from the parent body to the sensor (model) */
  inline const sva::PTransformd & X_p_f() const
  {
    return Device::X_p_s();
  }

  DevicePtr clone() const override;

  inline void setCameraId(int id)
  {
    id_ = id;
  }
  inline int getCameraId() const
  {
    return id_;
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  void capture();
  void release() const;

#ifdef WITH_ROS
  CameraDevice(const std::string & name,
               const std::string & parentBodyName,
               const sva::PTransformd & X_p_f,
               const std::string & topic,
               rclcpp::Node::SharedPtr & node);

  void setTopic(const std::string & topic);
#endif

private:
#ifdef WITH_ROS
  image_transport::Subscriber image_sub_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  rclcpp::Node::SharedPtr node_;
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
#endif

  void startCaptureThread();

  void open();

private:
  std::mutex image_mtx_;
  cv::Mat image_;
  int id_;

  std::thread capture_thread_;
  std::thread display_thread_;
  bool is_display_;
  bool stop_capture_;

  mutable cv::VideoCapture cam_;
};

} // namespace mc_rbdyn
