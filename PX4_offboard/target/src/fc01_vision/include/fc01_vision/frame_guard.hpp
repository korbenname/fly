#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace fc01_vision {

// 单个采集来源的测量身份。源时刻用于拒绝旧图/未来图；monotonic用于检测断流。
// 接收重复图不会修改last_receive，因此定时器重复发布也无法续鲜。
class FrameGuard {
public:
  explicit FrameGuard(double timeout_s = .3, double future_s = .05)
      : timeout_s_(timeout_s), future_s_(future_s) {
  }

  bool observe(std::int64_t stamp_ns, std::uint64_t sequence, std::int64_t source_now_ns,
               double receive_mono) {
    if (!std::isfinite(receive_mono) || receive_mono < last_receive_) {
      reason_ = "RECEIVE_CLOCK_REGRESSED";
      valid_ = false;
      return false;
    }
    const double age = (static_cast<double>(source_now_ns) - static_cast<double>(stamp_ns)) * 1e-9;
    if (stamp_ns <= 0 || source_now_ns <= 0 || sequence == 0 || age > timeout_s_ ||
        age < -future_s_) {
      reason_ = age < -future_s_ ? "SOURCE_TIME_FUTURE" : "SOURCE_TIME_STALE_OR_INVALID";
      valid_ = false;
      return false;
    }
    if (stamp_ns == last_stamp_ || sequence == last_sequence_) {
      reason_ = "DUPLICATE_FRAME";
      valid_ = false;
      return false;
    }
    if (last_stamp_ != 0 && (stamp_ns < last_stamp_ || sequence < last_sequence_)) {
      ++reset_counter_;
      last_stamp_ = stamp_ns;
      last_sequence_ = sequence;
      reason_ = "SOURCE_RESET_OR_TIME_REGRESSION";
      valid_ = false;
      return false;
    }
    last_stamp_ = stamp_ns;
    last_sequence_ = sequence;
    last_receive_ = receive_mono;
    valid_ = true;
    reason_ = "FRAME_CURRENT";
    return true;
  }

  bool current(std::int64_t source_now_ns, double receive_now) const {
    const double age =
        (static_cast<double>(source_now_ns) - static_cast<double>(last_stamp_)) * 1e-9;
    return valid_ && receive_now >= last_receive_ && receive_now - last_receive_ <= timeout_s_ &&
           age >= -future_s_ && age <= timeout_s_;
  }
  void invalidate(const std::string& reason) {
    valid_ = false;
    reason_ = reason;
  }
  void resetSource() {
    ++reset_counter_;
    valid_ = false;
    last_stamp_ = 0;
    last_sequence_ = 0;
    reason_ = "CAMERA_RECONNECTED";
  }
  std::uint32_t resetCounter() const {
    return reset_counter_;
  }
  std::int64_t stamp() const {
    return last_stamp_;
  }
  std::uint64_t sequence() const {
    return last_sequence_;
  }
  const std::string& reason() const {
    return reason_;
  }

private:
  double timeout_s_{.3}, future_s_{.05}, last_receive_{0.};
  std::int64_t last_stamp_{0};
  std::uint64_t last_sequence_{0};
  std::uint32_t reset_counter_{0};
  bool valid_{false};
  std::string reason_{"NO_FRAME"};
};

} // namespace fc01_vision
