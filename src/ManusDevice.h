#pragma once

#include <mc_rbdyn/Device.h>
#include <mc_rtc/gui.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <manus_ros2_msgs/msg/manus_glove.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mc_rbdyn
{

struct ManusDevice : public Device
{
  struct RawNode
  {
    int nodeId{0};
    int parentNodeId{0};
    std::string jointType;
    std::string chainType;
    sva::PTransformd pose{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  };

  struct Ergonomics
  {
    std::string type;
    double value{0.};
  };

  struct RawSensor
  {
    int sensorId{0};
    sva::PTransformd pose{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  };

  struct Data
  {
    int gloveId{-1};
    std::string side;
    std::vector<RawNode> rawNodes;
    std::vector<Ergonomics> ergonomics;
    std::optional<Eigen::Quaterniond> wristOrientation;
    std::vector<RawSensor> rawSensors;
    std::unordered_map<std::string, std::vector<Ergonomics>> fingers;
    std::chrono::steady_clock::time_point stamp{};
  };

  ~ManusDevice() noexcept override = default;

  ManusDevice(const ManusDevice & other);
  ManusDevice & operator=(const ManusDevice & other);

  ManusDevice(ManusDevice &&) noexcept = default;
  ManusDevice & operator=(ManusDevice &&) noexcept = default;

  ManusDevice(const std::string & name);

  ManusDevice(const std::string & name, const std::string & topic, rclcpp::Node::SharedPtr & node);

  DevicePtr clone() const override;

  const ManusDevice::Data & data() const;

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

private:
  mutable std::mutex dataMutex_;
  Data data_;

  void gloveCallback(const manus_ros2_msgs::msg::ManusGlove::SharedPtr msg);

  std::string topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<manus_ros2_msgs::msg::ManusGlove>::SharedPtr sub_;
};

} // namespace mc_rbdyn
