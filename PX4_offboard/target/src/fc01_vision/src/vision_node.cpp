#include "fc01_vision/vision_node.hpp"

#include <algorithm>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <opencv2/imgproc.hpp>

#include "fc01_vision/param_utils.hpp"

namespace fc01_vision {
namespace {

int64_t systemNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

} // namespace

VisionNode::VisionNode(const rclcpp::NodeOptions& options) : rclcpp::Node("fc01_vision", options) {
  loadParameters();

  // 目标观测只保留最新一帧：BEST_EFFORT、VOLATILE、depth 1。
  rclcpp::QoS state_qos(rclcpp::KeepLast(1));
  state_qos.best_effort().durability_volatile();
  state_pub_ = create_publisher<jikl_fc01_system::msg::TargetObservation>(state_topic_, state_qos);

  if (publish_debug_image_) {
    // RELIABLE so desktop viewers (rqt_image_view subscribes reliably) can
    // display it; a reliable publisher still feeds best-effort subscribers.
    rclcpp::QoS debug_qos(rclcpp::KeepLast(1));
    debug_qos.reliable().durability_volatile();
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("debug_image_topic", "/fc01/vision/debug_image"), debug_qos);
  }

  pipeline_ = std::make_unique<VisionPipeline>(detector_config_, estimator_config_, filter_config_);

  camera_ = std::make_unique<CameraSource>(camera_config_, [this](const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  });

  const double rate = std::max(1.0, publish_rate_hz_);
  publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {
    publishOnce();
  });

  status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/fc01/vision/camera_status"), 1);
  // 所有会抛异常的端点/参数先完成，最后才启动线程；失败时显式收回已启动线程。
  try {
    camera_->start();
    startWorker();
  } catch (...) {
    worker_running_ = false;
    camera_->stop();
    if (worker_.joinable())
      worker_.join();
    throw;
  }

  RCLCPP_INFO(get_logger(),
              "publishing TargetObservation on %s at %.1f Hz (BEST_EFFORT, "
              "VOLATILE, depth 1)",
              state_topic_.c_str(), rate);
  RCLCPP_INFO(get_logger(),
              "target: shape=%s rect=%.3fx%.3f m circle_d=%.3f m; "
              "intrinsics fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
              detector_config_.shape.c_str(), estimator_config_.target_width_m,
              estimator_config_.target_height_m, estimator_config_.target_diameter_m,
              intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy);
  if (!intrinsics_.valid()) {
    RCLCPP_WARN(get_logger(), "camera intrinsics are invalid; position output will be "
                              "invalid until camera_fx is configured");
  }
}

VisionNode::~VisionNode() {
  worker_running_ = false;
  if (camera_) {
    camera_->stop(); // also wakes waitNext()
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

void VisionNode::loadParameters() {
  camera_config_.device_index = static_cast<int>(declare_parameter<int>("device_index", 0));
  camera_config_.device_path = declare_parameter<std::string>("device_path", "");
  camera_config_.width = static_cast<int>(declare_parameter<int>("frame_width", 640));
  camera_config_.height = static_cast<int>(declare_parameter<int>("frame_height", 480));
  camera_config_.fps = static_cast<int>(declare_parameter<int>("frame_fps", 30));
  camera_config_.use_mjpg = declare_parameter<bool>("use_mjpg", true);
  camera_config_.flip_horizontal = declare_parameter<bool>("flip_horizontal", false);
  // V4L2 buffer count. The default 1 is fine at 30 fps, but this UVC camera
  // halves its rate (120 -> 60 fps) and returns only every other frame when
  // the driver is given a single buffer; 4 restores the full 120 fps.
  camera_config_.buffer_size =
      std::max(1, static_cast<int>(declare_parameter<int>("buffer_size", 1)));

  detector_config_ = declareDetectorConfig(*this);
  estimator_config_ = declareEstimatorConfig(*this);
  filter_config_ = declareFilterConfig(*this);

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
  detect_every_n_ = std::max(1, static_cast<int>(declare_parameter<int>("detect_every_n", 1)));
  stale_timeout_s_ = filter_config_.stale_timeout_s;
  frame_id_ = declare_parameter<std::string>("frame_id", "body_frd");
  state_topic_ = declare_parameter<std::string>("state_topic", "/fc01/vision/target_observation");
  // One switch for the live preview/slider window and the annotated debug
  // image. The individual parameters still work and are OR-ed with it, so a
  // headless deployment can keep everything off in the config file.
  const bool visualization = declare_parameter<bool>("visualization", false);
  publish_debug_image_ = visualization || declare_parameter<bool>("publish_debug_image", false);
  show_tuning_window_ = visualization || declare_parameter<bool>("show_tuning_window", false);
  // Empty disables saving. When set, the tuning window writes the current
  // slider values there on every change and on shutdown.
  tuning_output_path_ = declare_parameter<std::string>("tuning_output_path", "");

  intrinsics_ = declareIntrinsics(*this, camera_config_.width, camera_config_.height);

  adapter_config_ = declareAdapterConfig(*this, "fc01_vision", intrinsics_, estimator_config_);
  if (!std::isfinite(stale_timeout_s_) || stale_timeout_s_ <= .01 ||
      adapter_config_.capture_latency_bound_s >= stale_timeout_s_)
    throw std::invalid_argument("camera latency bound must fit finite frame freshness budget");
  frame_guard_ = FrameGuard(stale_timeout_s_ - adapter_config_.capture_latency_bound_s);
}

void VisionNode::startWorker() {
  worker_running_ = true;
  worker_ = std::thread([this]() {
    workerLoop();
  });
}

void VisionNode::workerLoop() {
  try {
    while (worker_running_ && rclcpp::ok()) {
      cv::Mat frame;
      int64_t stamp_ns = 0;
      uint64_t seq = 0;
      if (!camera_->waitNext(frame, stamp_ns, seq, 200)) {
        continue;
      }
      if (detect_every_n_ > 1 && (seq % static_cast<uint64_t>(detect_every_n_)) != 0) {
        continue;
      }

      // Highgui lives in this thread; create the window on the first frame so
      // a headless run fails gracefully instead of at startup.
      if (show_tuning_window_ && !tuning_) {
        auto window = std::make_unique<TuningWindow>();
        if (window->create("fc01 vision tuning", *pipeline_, tuning_output_path_)) {
          tuning_ = std::move(window);
        } else {
          show_tuning_window_ = false;
          RCLCPP_WARN(get_logger(), "show_tuning_window=true but no GUI is available; "
                                    "continuing without the slider window");
        }
      }
      if (tuning_) {
        tuning_->poll(*pipeline_);
      }

      const auto reset = camera_->resetCounter();
      const double mono =
          std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
              .count();
      bool frame_current = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (reset != camera_reset_) {
          camera_reset_ = reset;
          frame_guard_.resetSource();
          pipeline_->reset();
        }
        frame_current = frame_guard_.observe(stamp_ns, seq, systemNowNs(), mono);
        frame_reason_ = frame_guard_.reason();
      }
      if (!frame_current)
        continue;
      if (frame.cols != intrinsics_.width || frame.rows != intrinsics_.height) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        frame_guard_.invalidate("CAMERA_RESOLUTION_MISMATCH");
        frame_reason_ = "CAMERA_RESOLUTION_MISMATCH";
        continue;
      }
      TargetState state = pipeline_->process(frame, intrinsics_, stamp_ns);
      state.frame_seq = seq; // 保留采集序号，跳帧检测不重编造连续序号。
      cv::Mat debug;
      if (publish_debug_image_ || tuning_) {
        debug = pipeline_->drawDebug(frame);
      }
      if (tuning_) {
        tuning_->show(debug);
      }

      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = state;
      if (publish_debug_image_) {
        latest_debug_ = std::move(debug);
      }
    }
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    frame_guard_.invalidate("IMAGE_PROCESSING_FAILED");
    frame_reason_ = std::string("IMAGE_PROCESSING_FAILED: ") + error.what();
    worker_running_ = false;
    RCLCPP_ERROR(get_logger(), "%s", frame_reason_.c_str());
  }
}

void VisionNode::publishOnce() {
  TargetState state;
  cv::Mat debug;
  const int64_t now_ns = systemNowNs();
  const double mono =
      std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    if (publish_debug_image_)
      debug = latest_debug_;
    adapter_config_.reset_counter = frame_guard_.resetCounter();
    const bool fresh = frame_guard_.current(now_ns, mono) && camera_->connected() &&
                       state.stamp_ns == frame_guard_.stamp() &&
                       state.frame_seq == frame_guard_.sequence();
    reason = fresh ? (state.detected ? "FRAME_CURRENT" : "TARGET_NOT_DETECTED")
                   : "CAMERA_FRAME_STALE_OR_DISCONNECTED";
    if (!fresh) {
      if (!worker_running_ && frame_reason_.find("FAILED") != std::string::npos)
        reason = frame_reason_;
      state.detected = false;
      state.position_valid = false;
      state.velocity_valid = false;
    }
  }
  std_msgs::msg::String status;
  status.data = cameraStatus(adapter_config_, reason, "v4l2_read_completion_system",
                             camera_->lastReadSeconds());
  status_pub_->publish(status);

  const rclcpp::Time stamp = state.stamp_ns > 0 ? rclcpp::Time(state.stamp_ns, RCL_SYSTEM_TIME)
                                                : rclcpp::Time(int64_t{0}, RCL_SYSTEM_TIME);

  jikl_fc01_system::msg::TargetObservation message;
  fillTargetObservation(state, stamp, frame_id_, adapter_config_, message);
  state_pub_->publish(message);

  if (publish_debug_image_ && debug_pub_ && !debug.empty()) {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;
    cv_bridge::CvImage cv_image(header, "bgr8", debug);
    debug_pub_->publish(*cv_image.toImageMsg());
  }
}

} // namespace fc01_vision
