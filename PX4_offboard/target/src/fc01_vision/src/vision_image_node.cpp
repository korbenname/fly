// 图像驱动的目标检测节点：依赖统一jikl_fc01_system消息包。
// 仿真或相机驱动提供sensor_msgs/Image；输出相机中心原点的目标相对量，
// 不是自机NED里程计。需要先生成TargetObservation及其C++类型支持。
//
// Outputs (all best effort, volatile):
//   jikl_fc01_system/TargetObservation ~/state            primary contract output
//   geometry_msgs/PointStamped   ~/target_relative  FRD position, NaN if invalid
//   std_msgs/Bool                ~/target_valid
//   sensor_msgs/Image            ~/debug_image      optional
#include <cmath>
#include <algorithm>
#include <climits>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <jikl_fc01_system/msg/target_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "fc01_vision/frame_guard.hpp"

#include "fc01_vision/param_utils.hpp"
#include "fc01_vision/tuning_window.hpp"
#include "fc01_vision/vision_pipeline.hpp"
#include "fc01_vision/vision_state_adapter.hpp"

namespace fc01_vision {

class VisionImageNode : public rclcpp::Node {
public:
  VisionImageNode() : rclcpp::Node("fc01_vision_image") {
    const int fallback_width = static_cast<int>(declare_parameter<int>("frame_width", 640));
    const int fallback_height = static_cast<int>(declare_parameter<int>("frame_height", 480));

    const DetectorConfig detector_config = declareDetectorConfig(*this);
    const EstimatorConfig estimator_config = declareEstimatorConfig(*this);
    const FilterConfig filter_config = declareFilterConfig(*this);
    intrinsics_ = declareIntrinsics(*this, fallback_width, fallback_height);

    use_camera_info_ = declare_parameter<bool>("use_camera_info", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "body_frd");
    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    // OpenCV highgui window: live preview + trackbars for the detector.
    const bool show_tuning_window = declare_parameter<bool>("show_tuning_window", false);
    const std::string tuning_window_name =
        declare_parameter<std::string>("tuning_window_name", "fc01 vision tuning");
    const std::string tuning_output_path = declare_parameter<std::string>("tuning_output_path", "");

    const std::string image_topic =
        declare_parameter<std::string>("image_topic", "/sim/camera/image_raw");
    const std::string info_topic =
        declare_parameter<std::string>("camera_info_topic", "/sim/camera/camera_info");
    const std::string relative_topic =
        declare_parameter<std::string>("relative_topic", "/fc01/vision/target_relative");
    const std::string valid_topic =
        declare_parameter<std::string>("valid_topic", "/fc01/vision/target_valid");
    const std::string state_topic =
        declare_parameter<std::string>("state_topic", "/fc01/vision/target_observation");

    adapter_config_ =
        declareAdapterConfig(*this, "fc01_vision_image", intrinsics_, estimator_config);
    if (!std::isfinite(filter_config.stale_timeout_s) || filter_config.stale_timeout_s <= .01 ||
        adapter_config_.capture_latency_bound_s >= filter_config.stale_timeout_s)
      throw std::invalid_argument("camera latency bound must fit finite frame freshness budget");
    frame_guard_ =
        FrameGuard(filter_config.stale_timeout_s - adapter_config_.capture_latency_bound_s);

    pipeline_ = std::make_unique<VisionPipeline>(detector_config, estimator_config, filter_config);

    if (show_tuning_window) {
      // The trackbars cover exactly the parameters the detector uses; the
      // preview is the same annotated frame as the debug image.
      if (!tuning_.create(tuning_window_name, *pipeline_, tuning_output_path)) {
        RCLCPP_WARN(get_logger(), "show_tuning_window=true but no GUI is available; "
                                  "continuing without the slider window");
      } else {
        show_tuning_ = true;
        RCLCPP_INFO(get_logger(),
                    "tuning window '%s' opened: HSV threshold, blob filters, "
                    "target diameter",
                    tuning_window_name.c_str());
      }
    }

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          onImage(std::move(message));
        });

    if (use_camera_info_) {
      info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          info_topic, sensor_qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
            onCameraInfo(std::move(message));
          });
    }

    rclcpp::QoS output_qos(rclcpp::KeepLast(1));
    output_qos.best_effort().durability_volatile();
    relative_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(relative_topic, output_qos);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic, output_qos);
    state_pub_ =
        create_publisher<jikl_fc01_system::msg::TargetObservation>(state_topic, output_qos);

    status_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("status_topic", "/fc01/vision/camera_status"), 1);
    expiry_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      const auto now_ns = get_clock()->now().nanoseconds();
      const auto mono = monotonic();
      if (!frame_guard_.current(now_ns, mono)) {
        latest_state_.detected = latest_state_.position_valid = latest_state_.velocity_valid =
            false;
        publishState(latest_state_, "CAMERA_FRAME_STALE_OR_DISCONNECTED");
      }
    });

    if (publish_debug_image_) {
      // RELIABLE so every desktop viewer works: rqt_image_view subscribes with
      // a reliable profile, and a reliable publisher is also compatible with
      // best-effort subscribers. The main outputs stay BEST_EFFORT.
      rclcpp::QoS debug_qos(rclcpp::KeepLast(1));
      debug_qos.reliable().durability_volatile();
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
          declare_parameter<std::string>("debug_image_topic", "/fc01/vision/debug_image"),
          debug_qos);
    }

    RCLCPP_INFO(get_logger(),
                "image input %s, output state=%s relative=%s valid=%s; "
                "target shape=%s diameter=%.3f m; camera rotation configured",
                image_topic.c_str(), state_topic.c_str(), relative_topic.c_str(),
                valid_topic.c_str(), detector_config.shape.c_str(),
                estimator_config.target_diameter_m);
    if (!intrinsics_.valid()) {
      RCLCPP_WARN(get_logger(), "no valid intrinsics yet; waiting for CameraInfo on %s",
                  info_topic.c_str());
    }
  }

private:
  void rejectCameraInfo(const std::string& reason) {
    // 标定发生变化或无法解释时锁存失效。后续“好”CameraInfo不能悄悄恢复旧确认；
    // 必须核对现场标定配置并重启，防止同一任务混用两套相机模型。
    camera_info_fault_ = reason;
    adapter_config_.calibration_valid = false;
    adapter_config_.intrinsics_confirmed = false;
    frame_guard_.invalidate(reason);
    pipeline_->reset();
    latest_state_.detected = latest_state_.position_valid = latest_state_.velocity_valid = false;
    publishState(latest_state_, reason);
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 本估计器只有针孔+零畸变模型；不能把带畸变的raw图像直接当作已矫正图像。
    // 全K均须有限且符合当前模型，而不只检查fx/fy两个字段。
    const bool finite_k = std::all_of(msg->k.begin(), msg->k.end(),
                                     [](double x) { return std::isfinite(x); });
    if (!finite_k || msg->k[0] <= 0. || msg->k[4] <= 0. || msg->width == 0 ||
        msg->height == 0 || msg->width > INT_MAX || msg->height > INT_MAX ||
        msg->k[1] != 0. || msg->k[3] != 0. || msg->k[6] != 0. || msg->k[7] != 0. ||
        msg->k[8] != 1.) {
      rejectCameraInfo("CAMERA_INFO_INTRINSICS_INVALID");
      return;
    }
    const bool supported_model = msg->distortion_model.empty() ||
                                 msg->distortion_model == "plumb_bob" ||
                                 msg->distortion_model == "rational_polynomial";
    const bool zero_distortion = std::all_of(msg->d.begin(), msg->d.end(),
                                           [](double x) { return std::isfinite(x) && x == 0.; });
    const bool supported_length = msg->d.empty() ||
        (msg->distortion_model == "plumb_bob" && msg->d.size() == 5) ||
        (msg->distortion_model == "rational_polynomial" && msg->d.size() == 8);
    if (!supported_model || !zero_distortion || !supported_length) {
      rejectCameraInfo("CAMERA_INFO_DISTORTION_UNSUPPORTED");
      return;
    }
    if (intrinsics_.fx != msg->k[0] || intrinsics_.fy != msg->k[4] ||
        intrinsics_.cx != msg->k[2] || intrinsics_.cy != msg->k[5] ||
        intrinsics_.width != static_cast<int>(msg->width) ||
        intrinsics_.height != static_cast<int>(msg->height)) {
      rejectCameraInfo("CAMERA_INFO_CHANGED_RECONFIRM_REQUIRED");
      return;
    }
    // 仅与已确认参数完全一致的CameraInfo可证明配对；重复相同标定不续鲜图像，
    // 也不撤销当前新帧。CameraInfo本身不是动态测量，图像仍独立检查源/接收时间。
    if (!have_info_) {
      have_info_ = true;
      RCLCPP_INFO(get_logger(), "CameraInfo matches configured rectified pinhole profile (%dx%d)",
                  intrinsics_.width, intrinsics_.height);
    }
  }

  static double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  void publishState(const TargetState& state, const std::string& reason) {
    adapter_config_.reset_counter = frame_guard_.resetCounter();
    jikl_fc01_system::msg::TargetObservation output;
    fillTargetObservation(state, rclcpp::Time(std::max<int64_t>(0, state.stamp_ns)), frame_id_,
                          adapter_config_, output);
    state_pub_->publish(output);
    geometry_msgs::msg::PointStamped relative;
    relative.header.stamp = output.stamp;
    relative.header.frame_id = frame_id_;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    relative.point.x = output.tracking ? output.position[0] : nan;
    relative.point.y = output.tracking ? output.position[1] : nan;
    relative.point.z = output.tracking ? output.position[2] : nan;
    relative_pub_->publish(relative);
    std_msgs::msg::Bool valid;
    valid.data = output.tracking;
    valid_pub_->publish(valid);
    std_msgs::msg::String status;
    status.data = cameraStatus(adapter_config_, camera_info_fault_.empty() ? reason : camera_info_fault_,
                               "image_header_source");
    status_pub_->publish(status);
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    // Image在ROS2里没有sequence字段，以严格推进的原始stamp识别帧。
    // 相同stamp保持原seq；收到回退源会推动reset，并撤销该帧。
    if (stamp_ns != frame_guard_.stamp())
      ++image_sequence_;
    if (!frame_guard_.observe(stamp_ns, image_sequence_, get_clock()->now().nanoseconds(),
                              monotonic())) {
      TargetState rejected;
      rejected.stamp_ns = stamp_ns;
      rejected.frame_seq = image_sequence_;
      latest_state_ = rejected;
      publishState(latest_state_, frame_guard_.reason());
      if (frame_guard_.reason() == "SOURCE_RESET_OR_TIME_REGRESSION")
        pipeline_->reset();
      return;
    }
    if (!camera_info_fault_.empty() || (use_camera_info_ && !have_info_)) {
      latest_state_ = TargetState{};
      latest_state_.stamp_ns = stamp_ns;
      latest_state_.frame_seq = image_sequence_;
      publishState(latest_state_, camera_info_fault_.empty() ? "CAMERA_INFO_NOT_RECEIVED" : camera_info_fault_);
      return;
    }
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception& error) {
      frame_guard_.invalidate("IMAGE_CONVERSION_FAILED");
      latest_state_ = TargetState{};
      latest_state_.stamp_ns = stamp_ns;
      latest_state_.frame_seq = image_sequence_;
      publishState(latest_state_, "IMAGE_CONVERSION_FAILED");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge conversion failed: %s",
                           error.what());
      return;
    }
    if (cv_image->image.empty()) {
      frame_guard_.invalidate("IMAGE_EMPTY");
      latest_state_ = TargetState{};
      latest_state_.stamp_ns = stamp_ns;
      latest_state_.frame_seq = image_sequence_;
      publishState(latest_state_, "IMAGE_EMPTY");
      return;
    }
    if (cv_image->image.cols != intrinsics_.width || cv_image->image.rows != intrinsics_.height) {
      frame_guard_.invalidate("CAMERA_RESOLUTION_MISMATCH");
      latest_state_ = TargetState{};
      latest_state_.stamp_ns = stamp_ns;
      latest_state_.frame_seq = image_sequence_;
      publishState(latest_state_, "CAMERA_RESOLUTION_MISMATCH");
      return;
    }
    TargetState state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = pipeline_->process(cv_image->image, intrinsics_, stamp_ns);
    }
    state.frame_seq = image_sequence_;
    latest_state_ = state;
    publishState(state, state.detected ? "FRAME_CURRENT" : "TARGET_NOT_DETECTED");

    if (show_tuning_) {
      cv::Mat debug;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool changed = tuning_.poll(*pipeline_);
        if (changed) {
          // Re-run on the same frame so the preview reacts immediately.
          state = pipeline_->process(cv_image->image, intrinsics_,
                                     rclcpp::Time(msg->header.stamp).nanoseconds());
        }
        debug = pipeline_->drawDebug(cv_image->image);
      }
      tuning_.show(debug);
    }

    if (debug_pub_) {
      cv::Mat debug;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        debug = pipeline_->drawDebug(cv_image->image);
      }
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
    }

    if (++frame_count_ % 60 == 0) {
      RCLCPP_INFO(get_logger(),
                  "frames=%llu detected=%d valid=%d range=%.2f "
                  "pos_frd=(%.2f, %.2f, %.2f) %s",
                  static_cast<unsigned long long>(frame_count_), state.detected ? 1 : 0,
                  state.position_valid ? 1 : 0, state.distance, state.x, state.y, state.z,
                  state.reason.c_str());
    }
  }

  CameraIntrinsics intrinsics_;
  bool use_camera_info_{true};
  bool have_info_{false};
  std::string camera_info_fault_;
  bool publish_debug_image_{false};
  std::string frame_id_;
  FrameGuard frame_guard_;
  TargetState latest_state_;
  uint64_t image_sequence_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  std::mutex mutex_;

  std::unique_ptr<VisionPipeline> pipeline_;
  TuningWindow tuning_;
  bool show_tuning_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr relative_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<jikl_fc01_system::msg::TargetObservation>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  AdapterConfig adapter_config_;
  uint64_t frame_count_{0};
};

} // namespace fc01_vision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fc01_vision::VisionImageNode>());
  rclcpp::shutdown();
  return 0;
}
