#include "ManusDevice.h"

#include <mc_rtc/gui/Label.h>

namespace mc_rbdyn
{

namespace
{
inline sva::PTransformd toTransform(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
  Eigen::Vector3d t{pose.position.x, pose.position.y, pose.position.z};
  return sva::PTransformd{q.normalized().toRotationMatrix(), t};
}
} // namespace

ManusDevice::ManusDevice(const std::string & name) : Device(name)
{
  type_ = "ManusGlove";
}

ManusDevice::ManusDevice(const ManusDevice & other) : ManusDevice(other.name())
{
  std::lock_guard<std::mutex> lock(other.dataMutex_);
  data_ = other.data_;
  topic_ = other.topic_;
  node_ = other.node_;
  if(node_ && !topic_.empty())
  {
    sub_ = node_->create_subscription<manus_ros2_msgs::msg::ManusGlove>(
        topic_, rclcpp::SensorDataQoS(), std::bind(&ManusDevice::gloveCallback, this, std::placeholders::_1));
  }
}

ManusDevice::ManusDevice(const std::string & name, const std::string & topic, rclcpp::Node::SharedPtr & node)
: ManusDevice(name)
{
  topic_ = topic;
  node_ = node;
  if(!node_)
  {
    mc_rtc::log::error_and_throw("[ManusDevice] Cannot create ROS subscription without a valid node");
  }
  sub_ = node_->create_subscription<manus_ros2_msgs::msg::ManusGlove>(
      topic_, rclcpp::SensorDataQoS(), std::bind(&ManusDevice::gloveCallback, this, std::placeholders::_1));
}

ManusDevice & ManusDevice::operator=(const ManusDevice & other)
{
  if(this == &other)
  {
    return *this;
  }
  name_ = other.name_;
  parent_ = other.parent_;
  type_ = other.type_;
  {
    std::lock_guard<std::mutex> lock_other(other.dataMutex_);
    std::lock_guard<std::mutex> lock_self(dataMutex_);
    data_ = other.data_;
  }
#ifdef WITH_ROS
  topic_ = other.topic_;
  node_ = other.node_;
  if(node_ && !topic_.empty())
  {
    sub_ = node_->create_subscription<manus_ros2_msgs::msg::ManusGlove>(
        topic_, rclcpp::SensorDataQoS(), std::bind(&ManusDevice::gloveCallback, this, std::placeholders::_1));
  }
  else
  {
    sub_.reset();
  }
#endif
  return *this;
}

DevicePtr ManusDevice::clone() const
{
  return DevicePtr(new ManusDevice(*this));
}

const ManusDevice::Data & ManusDevice::data() const
{
  std::lock_guard<std::mutex> lock(dataMutex_);
  return data_;
}

void ManusDevice::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"ManusPlugin", name()}, mc_rtc::gui::Label("Side", [this]() { return data().side; }),
                 mc_rtc::gui::Label("Raw nodes", [this]() { return std::to_string(data().rawNodes.size()); }),
                 mc_rtc::gui::Label("Ergonomics", [this]() { return std::to_string(data().ergonomics.size()); }));
  gui.addElement({"ManusPlugin", name(), "Data"},
                 mc_rtc::gui::Table("Raw Nodes", {"Node ID", "Pose"},
                                    [this]()
                                    {
                                      const auto & d = data();
                                      std::vector<std::tuple<int, std::string, std::string>> table;
                                      for(const auto & node : d.rawNodes)
                                      {
                                        table.push_back(
                                            {node.nodeId, node.chainType,
                                             fmt::format("{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}",
                                                         node.pose.translation().x(), node.pose.translation().y(),
                                                         node.pose.translation().z(),
                                                         node.pose.rotation().eulerAngles(0, 1, 2).x(),
                                                         node.pose.rotation().eulerAngles(0, 1, 2).y(),
                                                         node.pose.rotation().eulerAngles(0, 1, 2).z())});
                                      }
                                      return table;
                                    }));

  gui.addElement({"ManusPlugin", name(), "DataErgonomics"},
                 mc_rtc::gui::Table("Ergonomics", {"Finger", "Flange", "Value"},
                                    [this]()
                                    {
                                      const auto & d = data();
                                      std::vector<std::tuple<std::string, std::string, std::string>> table;
                                      table.reserve(d.ergonomics.size());
                                      for(const auto & node : d.fingers)
                                      {
                                        for(const auto & f : node.second)
                                          table.emplace_back(node.first, f.type, fmt::format("{:.2f}", f.value));
                                      }
                                      return table;
                                    }));
}

void ManusDevice::gloveCallback(const manus_ros2_msgs::msg::ManusGlove::SharedPtr msg)
{
  Data sample;
  sample.gloveId = msg->glove_id;
  sample.side = msg->side;
  sample.rawNodes.reserve(msg->raw_nodes.size());
  for(size_t i = 0; i < msg->raw_nodes.size(); ++i)
  {
    const auto & node = msg->raw_nodes[i];
    RawNode dst;
    dst.nodeId = node.node_id;
    dst.parentNodeId = node.parent_node_id;
    dst.jointType = node.joint_type;
    dst.chainType = node.chain_type;
    dst.pose = toTransform(node.pose);
    sample.rawNodes.push_back(std::move(dst));
    sample.fingers[node.chain_type].reserve(10);

    if(i < msg->ergonomics.size())
    {
      const auto & ergo = msg->ergonomics[i];
      sample.fingers[node.chain_type].push_back({ergo.type, ergo.value});
    }
  }
  sample.ergonomics.reserve(msg->ergonomics.size());
  for(const auto & ergo : msg->ergonomics)
  {
    sample.ergonomics.push_back({ergo.type, ergo.value});
  }
  if(msg->raw_sensor_count > 0)
  {
    sample.rawSensors.reserve(msg->raw_sensor.size());
    for(size_t idx = 0; idx < msg->raw_sensor.size(); ++idx)
    {
      RawSensor rs;
      rs.sensorId = static_cast<int>(idx);
      rs.pose = toTransform(msg->raw_sensor[idx]);
      sample.rawSensors.push_back(std::move(rs));
    }
    sample.wristOrientation = Eigen::Quaterniond(msg->raw_sensor_orientation.w, msg->raw_sensor_orientation.x,
                                                 msg->raw_sensor_orientation.y, msg->raw_sensor_orientation.z);
  }
  sample.stamp = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(dataMutex_);
  data_ = std::move(sample);
}

} // namespace mc_rbdyn
