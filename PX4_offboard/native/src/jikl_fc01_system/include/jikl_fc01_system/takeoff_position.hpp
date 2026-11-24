#pragma once

#include <cmath>
#include <cstdint>

namespace fc01::attitude {
// PX4 局部 NED 坐标，单位 m 和 m/s。frame 记录原点与水平估计器的重置身份。
struct PositionFrame {
  std::uint64_t reference_stamp{}, writer_generation{};
  std::uint8_t xy_reset{}, vxy_reset{}, heading_reset{};
  bool operator==(const PositionFrame& other) const {
    return reference_stamp == other.reference_stamp &&
           writer_generation == other.writer_generation && xy_reset == other.xy_reset &&
           vxy_reset == other.vxy_reset && heading_reset == other.heading_reset;
  }
};

struct TakeoffPositionSample {
  double x{}, y{}, vx{}, vy{};
  PositionFrame frame{};
  bool valid{};
  bool usable() const {
    return valid && frame.writer_generation && std::isfinite(x) && std::isfinite(y) &&
           std::isfinite(vx) && std::isfinite(vy);
  }
};

struct TakeoffPositionAnchor {
  double x{}, y{};
  PositionFrame frame{};
  bool captured{};
  bool matches(const TakeoffPositionSample& sample) const {
    return captured && sample.usable() && frame == sample.frame;
  }
  double error(const TakeoffPositionSample& sample) const {
    return std::hypot(sample.x - x, sample.y - y);
  }
};
}  // namespace fc01::attitude
