#include "fc01_vision/vision_state_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace fc01_vision {
namespace {
std::string quoted(const std::string& value) {
  std::string out = "\"";
  for (unsigned char c : value) {
    if (c == '\\' || c == '"') {
      out += '\\';
      out += char(c);
    } else if (c >= 32)
      out += char(c);
  }
  return out + '"';
}
} // namespace

cv::Matx33d rotationFromWxyz(const std::vector<double>& q) {
  if (q.size() != 4)
    throw std::invalid_argument("camera_to_body_wxyz needs four values");
  double norm = 0.;
  for (double x : q) {
    if (!std::isfinite(x))
      throw std::invalid_argument("nonfinite camera rotation");
    norm += x * x;
  }
  if (std::abs(norm - 1.) > 1e-3)
    throw std::invalid_argument("camera quaternion must be normalized");
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
          2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

AdapterConfig declareAdapterConfig(rclcpp::Node& node, const std::string& source,
                                   const CameraIntrinsics& intrinsics,
                                   const EstimatorConfig& estimator) {
  AdapterConfig c;
  const auto base = node.declare_parameter<std::string>("source_id", source);
  // 源身份带进程会话，重启不伪装为同一序号周期；reopen另通过reset_counter表示。
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  c.source_id = base + "@" + std::to_string(getpid()) + "-" + std::to_string(tick);
  c.body_frame_id = node.declare_parameter<std::string>("body_frame_id", "body_frd");
  c.intrinsics_confirmed = node.declare_parameter<bool>("calibration_confirmed", false);
  c.image_rectified_confirmed = node.declare_parameter<bool>("image_rectified_confirmed", false);
  c.calibration_id = node.declare_parameter<std::string>("calibration_id", "");
  c.extrinsics_confirmed = node.declare_parameter<bool>("extrinsics_confirmed", false);
  c.extrinsics_id = node.declare_parameter<std::string>("extrinsics_id", "");
  c.target_model_confirmed = node.declare_parameter<bool>("target_model_confirmed", false);
  c.target_model_id = node.declare_parameter<std::string>("target_model_id", "");
  c.capture_latency_confirmed = node.declare_parameter<bool>("capture_latency_confirmed", false);
  c.capture_latency_bound_s = node.declare_parameter<double>("capture_latency_bound_s", 0.0);
  c.invalid_position_std_m = node.declare_parameter<double>("invalid_position_std_m", 1000.0);
  c.invalid_velocity_std_mps = node.declare_parameter<double>("invalid_velocity_std_mps", 1000.0);
  auto q = node.declare_parameter<std::vector<double>>("camera_to_body_wxyz", {1., 0., 0., 0.});
  // 兼容旧pitch参数；若显式非零，必须与新旋转互斥，两个入口均旋转位置/速度/协方差。
  const double pitch =
      node.declare_parameter<double>("camera_pitch_deg", 0.0) * 3.14159265358979323846 / 180.;
  if (!std::isfinite(pitch))
    throw std::invalid_argument("nonfinite camera_pitch_deg");
  if (pitch != 0.) {
    if (q != std::vector<double>({1., 0., 0., 0.}))
      throw std::invalid_argument("use camera quaternion OR legacy pitch");
    q = {std::cos(pitch / 2.), 0., std::sin(pitch / 2.), 0.};
  }
  c.camera_to_body = rotationFromWxyz(q);
  if (base.empty() || base.size() > 128 || c.calibration_id.size() > 128 ||
      c.extrinsics_id.size() > 128 || c.target_model_id.size() > 128)
    throw std::invalid_argument("camera identity must be nonempty/bounded");
  if (!std::isfinite(c.invalid_position_std_m) || !std::isfinite(c.invalid_velocity_std_mps) ||
      c.invalid_position_std_m <= 0. || c.invalid_velocity_std_mps <= 0. ||
      c.invalid_position_std_m > 1e6 || c.invalid_velocity_std_mps > 1e6)
    throw std::invalid_argument("invalid camera covariance fallback");
  const bool model = std::isfinite(estimator.target_diameter_m) &&
                     std::isfinite(estimator.target_width_m) &&
                     std::isfinite(estimator.target_height_m) &&
                     (estimator.target_diameter_m > 0. ||
                      (estimator.target_width_m > 0. && estimator.target_height_m > 0.));
  c.intrinsics_confirmed =
      c.intrinsics_confirmed && intrinsics.valid() && !c.calibration_id.empty();
  c.target_model_confirmed = c.target_model_confirmed && model && !c.target_model_id.empty();
  c.extrinsics_confirmed =
      c.extrinsics_confirmed && !c.extrinsics_id.empty() && c.body_frame_id == "body_frd";
  c.capture_latency_confirmed = c.capture_latency_confirmed &&
                                std::isfinite(c.capture_latency_bound_s) &&
                                c.capture_latency_bound_s > 0. && c.capture_latency_bound_s <= .3;
  c.calibration_valid = configurationReason(c) == "CONFIGURATION_CONFIRMED";
  return c;
}

std::string configurationReason(const AdapterConfig& c) {
  if (!c.intrinsics_confirmed)
    return "CAMERA_CALIBRATION_UNCONFIRMED";
  if (!c.image_rectified_confirmed)
    return "CAMERA_RECTIFICATION_UNCONFIRMED";
  if (!c.extrinsics_confirmed)
    return "CAMERA_EXTRINSICS_UNCONFIRMED";
  if (!c.target_model_confirmed)
    return "TARGET_MODEL_UNCONFIRMED";
  if (!c.capture_latency_confirmed)
    return "CAPTURE_LATENCY_UNCONFIRMED";
  return "CONFIGURATION_CONFIRMED";
}

std::string cameraStatus(const AdapterConfig& c, const std::string& reason,
                         const std::string& stamp_kind, double read_seconds) {
  std::ostringstream out;
  out << "{\"ready\":" << (c.calibration_valid && reason == "FRAME_CURRENT" ? "true" : "false")
      << ",\"reason\":" << quoted(reason) << ",\"configuration\":" << quoted(configurationReason(c))
      << ",\"source_id\":" << quoted(c.source_id) << ",\"reset_counter\":" << c.reset_counter
      << ",\"calibration_id\":" << quoted(c.calibration_id)
      << ",\"extrinsics_id\":" << quoted(c.extrinsics_id)
      << ",\"target_model_id\":" << quoted(c.target_model_id)
      << ",\"stamp_kind\":" << quoted(stamp_kind)
      << ",\"read_seconds\":" << (std::isfinite(read_seconds) ? read_seconds : 0.)
      << ",\"missing\":[";
  bool first = true;
  for (const auto& item : std::vector<std::pair<bool, std::string>>{
           {c.intrinsics_confirmed, "CAMERA_CALIBRATION"},
           {c.image_rectified_confirmed, "CAMERA_RECTIFICATION"},
           {c.extrinsics_confirmed, "CAMERA_EXTRINSICS"},
           {c.target_model_confirmed, "TARGET_MODEL"},
           {c.capture_latency_confirmed, "CAPTURE_LATENCY"}}) {
    if (!item.first) {
      if (!first)
        out << ',';
      out << quoted(item.second);
      first = false;
    }
  }
  out << "],\"exposure_timestamp_measured\":false}";
  return out.str();
}

void fillTargetObservation(const TargetState& s, const rclcpp::Time& stamp,
                           const std::string& frame_id, const AdapterConfig& c,
                           jikl_fc01_system::msg::TargetObservation& message) {
  using Message = jikl_fc01_system::msg::TargetObservation;
  message = Message{};
  message.schema_version = 1;
  message.sequence = s.frame_seq;
  message.stamp = stamp;
  message.reset_counter = c.reset_counter;
  message.frame_id = frame_id;
  message.body_frame_id = c.body_frame_id;
  message.source_id = c.source_id;
  message.initialized = s.stamp_ns > 0;
  message.calibration_valid = c.calibration_valid;
  bool finite = std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.z) &&
                std::isfinite(s.vx) && std::isfinite(s.vy) && std::isfinite(s.vz);
  for (double x : s.cov)
    finite = finite && std::isfinite(x) && x >= 0.;
  const bool valid =
      s.detected && s.position_valid && finite && c.calibration_valid && frame_id == "body_frd";
  message.tracking = valid;
  const auto p = c.camera_to_body * cv::Vec3d(s.x, s.y, s.z);
  const auto v = c.camera_to_body * cv::Vec3d(s.vx, s.vy, s.vz);
  message.position = {p[0], p[1], p[2]};
  message.velocity = {v[0], v[1], v[2]};
  // 姿态/角速度/质量没有观测；默认单位四元数不获得有效位。
  message.attitude = {1., 0., 0., 0.};
  const double pvar = c.invalid_position_std_m * c.invalid_position_std_m;
  const double vvar = c.invalid_velocity_std_mps * c.invalid_velocity_std_mps;
  cv::Matx33d cp = cv::Matx33d::zeros(), cv = cv::Matx33d::zeros();
  for (int i = 0; i < 3; ++i) {
    cp(i, i) = valid ? s.cov[i] : pvar;
    cv(i, i) = valid ? s.cov[i + 3] : vvar;
  }
  cp = c.camera_to_body * cp * c.camera_to_body.t();
  cv = c.camera_to_body * cv * c.camera_to_body.t();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      message.pose_covariance[i * 6 + j] = cp(i, j);
      message.twist_covariance[i * 6 + j] = cv(i, j);
    }
  if (valid) {
    message.valid_fields = Message::POSITION | Message::POSE_COVARIANCE;
    if (s.velocity_valid)
      message.valid_fields |= Message::VELOCITY | Message::TWIST_COVARIANCE;
  }
}
} // namespace fc01_vision
