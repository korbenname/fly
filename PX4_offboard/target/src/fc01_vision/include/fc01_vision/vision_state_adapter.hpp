#pragma once

#include <cstdint>
#include <string>

#include <jikl_fc01_system/msg/target_observation.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/node.hpp>

#include "fc01_vision/types.hpp"

namespace fc01_vision {

struct AdapterConfig {
  std::string source_id{"fc01_vision"};
  std::string body_frame_id{"body_frd"};
  bool calibration_valid{false};
  bool intrinsics_confirmed{false};
  // 估计器使用零畸变模型，只有已矫正图像或已验证可用此模型时才能确认。
  bool image_rectified_confirmed{false};
  bool extrinsics_confirmed{false};
  bool target_model_confirmed{false};
  bool capture_latency_confirmed{false};
  double capture_latency_bound_s{0.0};
  std::string calibration_id, extrinsics_id, target_model_id;
  // 相机FRD分量转机体FRD分量。仅旋转，原点仍是相机中心；不在此处重复加杆臂。
  cv::Matx33d camera_to_body{cv::Matx33d::eye()};
  uint32_t reset_counter{0};
  double invalid_position_std_m{1000.0};
  double invalid_velocity_std_mps{1000.0};
};

// 两入口统一声明现场承诺；没有可查标定id/confirmed就不会输出有效测量。
AdapterConfig declareAdapterConfig(rclcpp::Node& node, const std::string& source,
                                   const CameraIntrinsics& intrinsics,
                                   const EstimatorConfig& estimator);
std::string configurationReason(const AdapterConfig& config);
cv::Matx33d rotationFromWxyz(const std::vector<double>& q);
std::string cameraStatus(const AdapterConfig& config, const std::string& reason,
                         const std::string& stamp_kind, double read_seconds = 0.0);

// The ONLY place that knows the TargetObservation field layout.
//
// What this vision pipeline can actually estimate:
//   POSITION / VELOCITY  target relative to the camera/body, body FRD axes.
//   POSE_COVARIANCE      linear (position) block from the alpha-beta filter.
//   TWIST_COVARIANCE     linear (velocity) block.
// What it cannot estimate, and therefore leaves with the validity bit clear:
//   ATTITUDE             written as identity [w=1, x=0, y=0, z=0].
//   ANGULAR_VELOCITY     written as zero.
//   QUALITY              written as zero.
//
// frame_id is the frame the position/velocity are expressed in (default
// "body_frd"). 该类型是相对目标；本节点未估计目标姿态，不能以单位四元数作有效姿态；
// this node publishes no attitude, so consumers must respect ATTITUDE.
void fillTargetObservation(const TargetState& state, const rclcpp::Time& stamp,
                           const std::string& frame_id, const AdapterConfig& config,
                           jikl_fc01_system::msg::TargetObservation& message);

} // namespace fc01_vision
