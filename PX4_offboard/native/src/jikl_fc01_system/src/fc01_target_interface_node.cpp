// 视觉接口只读接收器：验证真实DDS类型/话题/字段，不创建Runtime或任何PX4发布器。
// 这里“收到”只表示观察到报文；来源、时效、仲裁和飞行准入由后续阶段接入原生边界。
#include "jikl_fc01_system/protocol.hpp"
#include <jikl_fc01_system/msg/navigation_command.hpp>
#include <jikl_fc01_system/msg/target_observation.hpp>
#include <jikl_fc01_system/msg/velocity_command_v2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Velocity = jikl_fc01_system::msg::VelocityCommandV2;
using Navigation = jikl_fc01_system::msg::NavigationCommand;
using Observation = jikl_fc01_system::msg::TargetObservation;
std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}
const char* boolean(bool value) { return value ? "true" : "false"; }
template<class Values> std::string array(const Values& values) {
  std::string result = "[";
  for (const auto value : values) {
    if (result.size() > 1) result += ',';
    result += number(value);
  }
  return result + ']';
}
// 文本仅写入有界诊断副本；不截断后拿它判断“同一来源”。消息身份未在这里授信。
std::string text(const std::string& value) {
  return fc01::quote(value.size() <= 256 ? value : "<identity exceeds 256 bytes>");
}
template<class Message> std::string common(const Message& m) {
  return "\"schema_version\":" + std::to_string(m.schema_version)
      + ",\"sequence\":" + std::to_string(m.sequence)
      + ",\"stamp_ns\":" + std::to_string(std::int64_t(m.stamp.sec)*1000000000LL + m.stamp.nanosec)
      + ",\"reset_counter\":" + std::to_string(m.reset_counter)
      + ",\"source_id\":" + text(m.source_id) + ",\"frame_id\":" + text(m.frame_id);
}
std::string describe(const Velocity& m) {
  return '{' + common(m) + ",\"valid\":" + boolean(m.valid)
      + ",\"vx\":" + number(m.vx) + ",\"vy\":" + number(m.vy)
      + ",\"vz\":" + number(m.vz) + ",\"wz\":" + number(m.wz) + '}';
}
std::string describe(const Navigation& m) {
  return '{' + common(m) + ",\"valid\":" + boolean(m.valid)
      + ",\"mode\":" + std::to_string(m.mode)
      + ",\"position_valid\":" + boolean(m.position_valid)
      + ",\"yaw_valid\":" + boolean(m.yaw_valid)
      + ",\"sticks_valid\":" + boolean(m.sticks_valid)
      + ",\"position\":" + array(m.position) + ",\"yaw\":" + number(m.yaw)
      + ",\"sticks\":" + array(m.sticks) + '}';
}
std::string describe(const Observation& m) {
  return '{' + common(m) + ",\"body_frame_id\":" + text(m.body_frame_id)
      + ",\"valid_fields\":" + std::to_string(m.valid_fields)
      + ",\"initialized\":" + boolean(m.initialized) + ",\"tracking\":" + boolean(m.tracking)
      + ",\"calibration_valid\":" + boolean(m.calibration_valid)
      + ",\"position\":" + array(m.position) + ",\"velocity\":" + array(m.velocity)
      + ",\"attitude\":" + array(m.attitude)
      + ",\"angular_velocity_source\":" + text(m.angular_velocity_source)
      + ",\"angular_velocity\":" + array(m.angular_velocity)
      + ",\"pose_covariance\":" + array(m.pose_covariance)
      + ",\"twist_covariance\":" + array(m.twist_covariance)
      + ",\"quality\":" + number(m.quality) + '}';
}
class InterfaceNode final : public rclcpp::Node {
public:
  InterfaceNode() : Node("fc01_target_interface") {
    const auto velocity = topic("velocity_topic", "/fc01/vision/velocity_command_v2");
    const auto navigation = topic("navigation_topic", "/fc01/navigation/command");
    const auto target = topic("target_topic", "/fc01/vision/target_observation");
    const auto status = topic("status_topic", "/fc01/interface/status");
    if (velocity == navigation || velocity == target || velocity == status ||
        navigation == target || navigation == status || target == status)
      throw std::invalid_argument("interface topics must be distinct");
    // keep-last1限制排队；volatile不重放旧任务。可靠性匹配mission的reliable发布者。
    const auto commands = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto observations = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    velocity_ = create_subscription<Velocity>(velocity, commands, [this](Velocity::ConstSharedPtr m) {
      velocity_json_ = describe(*m); ++velocity_count_;
    });
    navigation_ = create_subscription<Navigation>(navigation, commands, [this](Navigation::ConstSharedPtr m) {
      navigation_json_ = describe(*m); ++navigation_count_;
    });
    target_ = create_subscription<Observation>(target, observations, [this](Observation::ConstSharedPtr m) {
      target_json_ = describe(*m); ++target_count_;
    });
    output_ = create_publisher<std_msgs::msg::String>(status, commands);
    // timer和回调均由单线程spin执行；只保留三个有界JSON，不积累消息日志。
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {
      std_msgs::msg::String out;
      out.data = "{\"kind\":\"target_interface\",\"control_authority\":false,\"counts\":{\"velocity\":"
          + std::to_string(velocity_count_) + ",\"navigation\":" + std::to_string(navigation_count_)
          + ",\"target\":" + std::to_string(target_count_) + "},\"velocity\":" + velocity_json_
          + ",\"navigation\":" + navigation_json_ + ",\"target\":" + target_json_
          + ",\"velocity_qos\":{\"depth\":" + std::to_string(velocity_->get_actual_qos().depth())
          + ",\"keep_last\":" + boolean(velocity_->get_actual_qos().history() == rclcpp::HistoryPolicy::KeepLast)
          + ",\"volatile\":" + boolean(velocity_->get_actual_qos().durability() == rclcpp::DurabilityPolicy::Volatile)
          + "}}";
      output_->publish(out);
    });
  }
private:
  std::string topic(const char* name, const char* fallback) {
    const auto value = declare_parameter<std::string>(name, fallback);
    if (value.empty() || value.size() > 256 || value.front() != '/' || value.rfind("/fmu/", 0) == 0)
      throw std::invalid_argument("invalid read-only interface topic");
    // 最终有效名由rclcpp校验；remap后的/fmu同样不能成为此节点的输出。
    const auto resolved = get_node_topics_interface()->resolve_topic_name(value);
    if (resolved.rfind("/fmu/", 0) == 0)
      throw std::invalid_argument("read-only interface cannot use PX4 topics");
    for (const auto& used : resolved_topics_)
      if (used == resolved)
        throw std::invalid_argument("remapped interface topics must be distinct");
    resolved_topics_.push_back(resolved);
    // 创建实体时rclcpp还会执行一次remap；传原名，避免把解析结果再次映射到/fmu。
    return value;
  }
  std::uint64_t velocity_count_{}, navigation_count_{}, target_count_{};
  std::vector<std::string> resolved_topics_;
  std::string velocity_json_{"null"}, navigation_json_{"null"}, target_json_{"null"};
  rclcpp::Subscription<Velocity>::SharedPtr velocity_;
  rclcpp::Subscription<Navigation>::SharedPtr navigation_;
  rclcpp::Subscription<Observation>::SharedPtr target_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<InterfaceNode>());
  } catch (const std::exception& error) {
    std::cerr << "FC01_TARGET_INTERFACE_FAILED: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
