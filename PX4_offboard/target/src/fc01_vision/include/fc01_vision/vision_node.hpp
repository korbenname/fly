#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "fc01_vision/frame_guard.hpp"

#include <jikl_fc01_system/msg/target_observation.hpp>

#include "fc01_vision/camera_source.hpp"
#include "fc01_vision/tuning_window.hpp"
#include "fc01_vision/types.hpp"
#include "fc01_vision/vision_pipeline.hpp"
#include "fc01_vision/vision_state_adapter.hpp"

namespace fc01_vision {

class VisionNode : public rclcpp::Node {
public:
  explicit VisionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~VisionNode() override;

private:
  void loadParameters();
  void startWorker();
  void workerLoop();
  void publishOnce();

  // Configuration
  CameraConfig camera_config_;
  DetectorConfig detector_config_;
  EstimatorConfig estimator_config_;
  FilterConfig filter_config_;
  CameraIntrinsics intrinsics_;
  AdapterConfig adapter_config_;
  FrameGuard frame_guard_;
  std::uint32_t camera_reset_{0};
  std::string frame_reason_{"NO_FRAME"};
  std::string frame_id_{"body_frd"};
  std::string state_topic_{"/fc01/vision/target_observation"};
  int detect_every_n_{1};
  double publish_rate_hz_{30.0};
  double stale_timeout_s_{0.5};
  bool publish_debug_image_{false};
  bool show_tuning_window_{false};
  std::string tuning_output_path_;

  // Runtime
  std::unique_ptr<CameraSource> camera_;
  std::unique_ptr<VisionPipeline> pipeline_;
  // Created inside the worker thread: OpenCV highgui windows must be created
  // and driven from one thread.
  std::unique_ptr<TuningWindow> tuning_;
  rclcpp::Publisher<jikl_fc01_system::msg::TargetObservation>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::thread worker_;
  std::atomic<bool> worker_running_{false};
  std::mutex state_mutex_;
  TargetState latest_state_;
  cv::Mat latest_debug_;
};

} // namespace fc01_vision
