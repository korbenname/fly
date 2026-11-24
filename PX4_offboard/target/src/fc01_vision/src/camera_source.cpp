#include "fc01_vision/camera_source.hpp"

#include <chrono>
#include <iostream>
#include <sstream>

#include <opencv2/videoio.hpp>

namespace fc01_vision {
namespace {

int64_t nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

} // namespace

CameraSource::CameraSource(CameraConfig config, LogFn log) : config_(config), log_(std::move(log)) {
}

CameraSource::~CameraSource() {
  stop();
}

void CameraSource::log(const std::string& message) const {
  if (log_) {
    log_(message);
  } else {
    std::cerr << "[camera] " << message << std::endl;
  }
}

bool CameraSource::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_)
    return true;
  running_ = true;
  try {
    thread_ = std::thread(&CameraSource::threadLoop, this);
  } catch (...) {
    running_ = false;
    throw;
  }
  return true;
}

void CameraSource::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool CameraSource::running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

int CameraSource::actualWidth() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_width_;
}

int CameraSource::actualHeight() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return actual_height_;
}

std::uint32_t CameraSource::resetCounter() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reset_counter_;
}
double CameraSource::lastReadSeconds() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_read_seconds_;
}
bool CameraSource::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

bool CameraSource::openCapture(cv::VideoCapture& cap) {
  cap.release();
  if (config_.device_path.empty())
    cap.open(config_.device_index, cv::CAP_V4L2);
  else
    cap.open(config_.device_path, cv::CAP_V4L2);
  if (!cap.isOpened()) {
    std::ostringstream oss;
    oss << "cannot open /dev/video" << config_.device_index;
    log(oss.str());
    return false;
  }

  if (config_.use_mjpg) {
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  cap.set(cv::CAP_PROP_FPS, config_.fps);
  cap.set(cv::CAP_PROP_BUFFERSIZE, config_.buffer_size);

  const int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  const int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  const double fps = cap.get(cv::CAP_PROP_FPS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    actual_width_ = w;
    actual_height_ = h;
  }

  std::ostringstream oss;
  oss << "opened /dev/video" << config_.device_index << " " << w << "x" << h << " @" << fps
      << " fps";
  if (w != config_.width || h != config_.height) {
    oss << " (requested " << config_.width << "x" << config_.height << "; check camera intrinsics)";
  }
  log(oss.str());
  return true;
}

void CameraSource::threadLoop() {
  cv::VideoCapture cap;
  bool opened = false;

  try {
    while (true) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
          break;
        }
      }

      if (!opened) {
        opened = openCapture(cap);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          connected_ = opened;
          if (opened) {
            ++reset_counter_;
            have_frame_ = false;
          }
        }
        if (!opened) {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          continue;
        }
      }

      cv::Mat frame;
      const auto read_started = std::chrono::steady_clock::now();
      if (!cap.read(frame) || frame.empty()) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          connected_ = false;
          have_frame_ = false;
        }
        log("frame read failed, reopening device");
        cap.release();
        opened = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      if (config_.flip_horizontal) {
        cv::flip(frame, frame, 1);
      }

      const int64_t stamp_ns = nowNs();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_read_seconds_ =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - read_started).count();
        latest_ = frame;
        latest_stamp_ns_ = stamp_ns;
        have_frame_ = true;
        ++seq_;
      }
      cv_.notify_all();
    }

  } catch (const std::exception& error) {
    log(std::string("CAMERA_CAPTURE_FAILED: ") + error.what());
  }
  cap.release();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    running_ = false;
  }
  cv_.notify_all();
}

bool CameraSource::waitNext(cv::Mat& frame, int64_t& stamp_ns, uint64_t& seq, int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex_);
  const bool got = cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] {
    return !running_ || seq_ != delivered_seq_;
  });
  if (!got) {
    return false;
  }
  if (seq_ == delivered_seq_) {
    return false; // stopped without a new frame
  }

  frame = latest_.clone();
  stamp_ns = latest_stamp_ns_;
  seq = seq_;
  delivered_seq_ = seq_;
  return true;
}

bool CameraSource::lastFrame(cv::Mat& frame, int64_t& stamp_ns, uint64_t& seq) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_frame_) {
    return false;
  }
  frame = latest_.clone();
  stamp_ns = latest_stamp_ns_;
  seq = seq_;
  return true;
}

} // namespace fc01_vision
