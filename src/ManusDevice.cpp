#include "ManusDevice.h"
#include <mc_rtc/gui/Checkbox.h>
#include <chrono>

namespace mc_rbdyn
{

ManusDevice::ManusDevice(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f)
: Device(name, parentBodyName, X_p_f), stop_capture_(false)
{
  type_ = "ManusDeviceSensor";
}

ManusDevice::~ManusDevice() noexcept
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

ManusDevice::ManusDevice() : ManusDevice("", "", sva::PTransformd::Identity()) {}

ManusDevice::ManusDevice(const ManusDevice & cd) : ManusDevice(cd.name(), cd.parentBody(), cd.X_p_f())
{
  id_ = cd.getManusId();

#ifdef WITH_ROS
  node_ = cd.node_;
  it_ = std::make_shared<image_transport::ImageTransport>(node_);

  if(cd.image_sub_.getTopic() != "")
  {
    image_sub_ = it_->subscribe(cd.image_sub_.getTopic(), 1,
                                std::bind(&ManusDevice::imageCallback, this, std::placeholders::_1));
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

ManusDevice::ManusDevice(const std::string & name,
                           const std::string & parentBodyName,
                           const sva::PTransformd & X_p_f,
                           int id)
: ManusDevice(name, parentBodyName, X_p_f)
{
  id_ = id;
  open();
  startCaptureThread();
}

ManusDevice & ManusDevice::operator=(const ManusDevice & cd)
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
      it_->subscribe(image_sub_.getTopic(), 1, std::bind(&ManusDevice::imageCallback, this, std::placeholders::_1));
#endif
  return *this;
}

DevicePtr ManusDevice::clone() const
{
  return DevicePtr(new ManusDevice(*this));
}

void ManusDevice::open()
{
  if(!cam_.open(id_))
  {
    mc_rtc::log::error_and_throw("[ManusPlugin::Manus] Manus {} cannot be openned with id: {}", name_, id_);
  }
}

void ManusDevice::startCaptureThread()
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

void ManusDevice::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"ManusPlugin"},
      mc_rtc::gui::Checkbox("Display " + name(), [&]() { return is_display_; }, [&]() { is_display_ = !is_display_; }));
}

void ManusDevice::capture()
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  if(cam_.isOpened())
  {
    cam_ >> image_;
  }
}

const cv::Mat & ManusDevice::getImage()
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  return image_;
}

void ManusDevice::setImage(const cv::Mat & image)
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  image_ = image;
}

void ManusDevice::release() const
{
  cam_.release();
}

#ifdef WITH_ROS
ManusDevice::ManusDevice(const std::string & name,
                           const std::string & parentBodyName,
                           const sva::PTransformd & X_p_f,
                           const std::string & topic,
                           bool use_compressed,
                           rclcpp::Node::SharedPtr & node)
: ManusDevice(name, parentBodyName, X_p_f)
{
  id_ = -1;
  node_ = node;
  it_ = std::make_shared<image_transport::ImageTransport>(node_);

  if(use_compressed)
  {
    image_transport::TransportHints hints(node_.get(), "compressed");
    image_sub_ =
        it_->subscribe(topic, 1, std::bind(&ManusDevice::imageCallback, this, std::placeholders::_1), nullptr, &hints);
  }
  else
  {
    image_sub_ = it_->subscribe(topic, 1, std::bind(&ManusDevice::imageCallback, this, std::placeholders::_1));
  }

  startCaptureThread();
}

void ManusDevice::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const std::lock_guard<std::mutex> lock(image_mtx_);
  image_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
}
#endif

} // namespace mc_rbdyn
