// 无硬件回归：测量身份、源/接收双时效和相机安装旋转的全部目标字段。
#include "fc01_vision/frame_guard.hpp"
#include "fc01_vision/vision_state_adapter.hpp"
#include "fc01_vision/vision_pipeline.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <cmath>

void require(bool v, const char* why) {
  if (!v)
    throw std::runtime_error(why);
}
void near(double a, double b) {
  require(std::abs(a - b) < 1e-9, "rotation/covariance mismatch");
}
int main() {
  using namespace fc01_vision;
  FrameGuard g(.3, .05);
  require(g.observe(1000000000, 1, 1000000000, 1.), "first frame");
  require(!g.observe(1000000000, 1, 1100000000, 1.1), "duplicate renewed frame");
  require(!g.current(1200000000, 1.2), "duplicate remains valid");
  require(g.observe(1200000000, 2, 1200000000, 1.2), "new frame");
  require(!g.current(1200000000, 1.51), "monotonic stall accepted");
  require(!g.observe(100000000, 3, 1200000000, 1.3), "old source accepted");
  require(!g.observe(1400000000, 3, 1200000000, 1.3), "future source accepted");
  require(!g.observe(1150000000, 3, 1200000000, 1.3), "regression accepted");
  require(g.resetCounter() == 1, "regression reset absent");
  require(g.observe(1250000000, 4, 1250000000, 1.4), "new after reset");
  require(!g.observe(1300000000, 5, 1300000000, 1.3), "receive clock regression");
  TargetState s;
  s.detected = s.position_valid = s.velocity_valid = true;
  s.stamp_ns = 1000000000;
  s.frame_seq = 7;
  s.x = 2.;
  s.y = 1.;
  s.z = .5;
  s.vx = .3;
  s.vy = .4;
  s.vz = .5;
  for (int i = 0; i < 6; ++i)
    s.cov[i] = i + 1.;
  AdapterConfig c;
  c.intrinsics_confirmed = true;
  require(configurationReason(c) == "CAMERA_RECTIFICATION_UNCONFIRMED",
          "zero distortion model implicitly confirmed");
  c.image_rectified_confirmed = true;
  require(configurationReason(c) == "CAMERA_EXTRINSICS_UNCONFIRMED",
          "rectification confirmation hides other missing fields");
  c.calibration_valid = true;
  const double h = std::sqrt(.5);
  c.camera_to_body = rotationFromWxyz({h, 0., 0., h});
  jikl_fc01_system::msg::TargetObservation m;
  fillTargetObservation(s, rclcpp::Time(s.stamp_ns), "body_frd", c, m);
  near(m.position[0], -1.);
  near(m.position[1], 2.);
  near(m.position[2], .5);
  near(m.velocity[0], -.4);
  near(m.velocity[1], .3);
  near(m.velocity[2], .5);
  near(m.pose_covariance[0], 2.);
  near(m.pose_covariance[7], 1.);
  near(m.twist_covariance[0], 5.);
  near(m.twist_covariance[7], 4.);
  c.camera_to_body = rotationFromWxyz({std::cos(.2), std::sin(.2), 0., 0.});
  fillTargetObservation(s, rclcpp::Time(s.stamp_ns), "body_frd", c, m);
  near(m.position[1], std::cos(.4) - .5 * std::sin(.4));
  near(m.velocity[1], .4 * std::cos(.4) - .5 * std::sin(.4));
  require(std::abs(m.pose_covariance[8]) > 0., "tilted covariance cross term lost");
  c.calibration_valid = false;
  fillTargetObservation(s, rclcpp::Time(s.stamp_ns), "body_frd", c, m);
  require(!m.tracking && m.valid_fields == 0, "unconfirmed calibration accepted");
  c.calibration_valid = true;
  s.x = NAN;
  fillTargetObservation(s, rclcpp::Time(s.stamp_ns), "body_frd", c, m);
  require(!m.tracking && m.valid_fields == 0, "nonfinite observation accepted");
  DetectorConfig d;
  d.ranges.push_back({0, 10, 80, 255, 50, 255});
  d.shape = "circle";
  EstimatorConfig e;
  e.target_diameter_m = .2;
  VisionPipeline p(d, e, {});
  CameraIntrinsics k;
  k.fx = k.fy = 400;
  k.cx = 320;
  k.cy = 240;
  k.width = 640;
  k.height = 480;
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::circle(image, {320, 240}, 40, {0, 0, 255}, -1);
  auto a = p.process(image, k, 1000000000);
  auto b = p.process(image, k, 1000000000);
  require(a.position_valid && !a.velocity_valid, "first frame velocity invented");
  require(!b.position_valid && b.frame_seq == a.frame_seq, "pipeline duplicate advanced");
  auto r = p.process(image, k, 900000000);
  require(!r.position_valid, "pipeline time regression");
  auto n = p.process(image, k, 1100000000);
  require(n.position_valid && n.velocity_valid, "new frame derivative");
  auto empty = p.process(cv::Mat{}, k, 1200000000);
  require(!empty.detected, "empty frame tracked");
  std::cout
      << "VISION_CONTRACT_SELFTEST_PASS frame_guard rotation covariance finite calibration pipeline\n";
}
