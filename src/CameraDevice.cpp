#include "CameraDevice.h"
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/logging.h>
#include <chrono>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <utility>

namespace mc_rbdyn
{

CameraDevice::CameraDevice(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f)
: Device(name, parentBodyName, X_p_f), stop_capture_(false)
{
  type_ = "CameraDeviceSensor";
}

CameraDevice::~CameraDevice() noexcept
{
  stop_capture_ = true;
  is_display_ = false;
  if(capture_thread_.joinable())
  {
    capture_thread_.join();
  }

  if(display_thread_.joinable())
  {
    display_thread_.join();
  }
};

CameraDevice::CameraDevice() : CameraDevice("", "", sva::PTransformd::Identity()) {}

CameraDevice::CameraDevice(const CameraDevice & cd) : CameraDevice(cd.name(), cd.parentBody(), cd.X_p_f())
{
  id_ = cd.getCameraId();

#ifdef WITH_ROS
  node_ = cd.node_;
  it_ = std::make_shared<image_transport::ImageTransport>(node_);

  if(cd.image_sub_.getTopic() != "")
  {
    image_sub_ = it_->subscribe(cd.image_sub_.getTopic(), 1,
                                std::bind(&CameraDevice::imageCallback, this, std::placeholders::_1));
  }
#endif

  if(cd.cam_.isOpened())
  {
    cd.release();
    open();
  }

  stop_capture_ = false;
  is_display_ = false;
  startCaptureThread();
}

CameraDevice::CameraDevice(const std::string & name,
                           const std::string & parentBodyName,
                           const sva::PTransformd & X_p_f,
                           int id)
: CameraDevice(name, parentBodyName, X_p_f)
{
  id_ = id;
  open();
  startCaptureThread();
}

CameraDevice & CameraDevice::operator=(const CameraDevice & cd)
{
  if(&cd == this)
  {
    return *this;
  }
  name_ = cd.name_;
  parent_ = cd.parent_;
  X_p_s_ = cd.X_p_s_;
  id_ = cd.id_;
#ifdef WITH_ROS
  node_ = cd.node_;
  it_ = std::make_shared<image_transport::ImageTransport>(node_);
  image_sub_ =
      it_->subscribe(image_sub_.getTopic(), 1, std::bind(&CameraDevice::imageCallback, this, std::placeholders::_1));
#endif
  return *this;
}

DevicePtr CameraDevice::clone() const
{
  return DevicePtr(new CameraDevice(*this));
}

void CameraDevice::open()
{
  if(!cam_.open(id_))
  {
    mc_rtc::log::error_and_throw("[CameraPlugin::Camera] Camera {} cannot be openned with id: {}", name_, id_);
  }
}

void CameraDevice::startCaptureThread()
{
  capture_thread_ = std::thread(
      [this]
      {
        while(!stop_capture_)
        {
          capture();
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      });

  display_thread_ = std::thread(
      [this]
      {
        bool window_created = false;
        is_display_ = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        while(!stop_capture_)
        {
          if(is_display_)
          {
            if(!window_created)
            {
              cv::namedWindow(name(), cv::WINDOW_AUTOSIZE);
              window_created = true;
            }

            auto & image = getImage();

            if(!image.empty())
            {
              cv::imshow(name(), image);
              cv::pollKey();
            }
          }
          else
          {
            if(window_created)
            {
              cv::destroyWindow(name());
              cv::waitKey(1);
              window_created = false;
            }
          }
        }
      });
}

void CameraDevice::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"CameraPlugin"},
      mc_rtc::gui::Checkbox("Display " + name(), [&]() { return is_display_; }, [&]() { is_display_ = !is_display_; }));
}

void CameraDevice::capture()
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  if(cam_.isOpened())
  {
    cam_ >> image_;
  }
}

const cv::Mat & CameraDevice::getImage()
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  return image_;
}

void CameraDevice::setImage(const cv::Mat & image)
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  image_ = image;
}

void CameraDevice::release() const
{
  cam_.release();
}

#ifdef WITH_ROS
CameraDevice::CameraDevice(const std::string & name,
                           const std::string & parentBodyName,
                           const sva::PTransformd & X_p_f,
                           const std::string & topic,
                           rclcpp::Node::SharedPtr & node)
: CameraDevice(name, parentBodyName, X_p_f)
{
  id_ = -1;
  node_ = node;
  it_ = std::make_shared<image_transport::ImageTransport>(node_);
  setTopic(topic);
  startCaptureThread();
}

void CameraDevice::setTopic(const std::string & topic)
{
  image_sub_ = it_->subscribe(topic, 1, std::bind(&CameraDevice::imageCallback, this, std::placeholders::_1));
}

void CameraDevice::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  image_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
}
#endif

} // namespace mc_rbdyn
