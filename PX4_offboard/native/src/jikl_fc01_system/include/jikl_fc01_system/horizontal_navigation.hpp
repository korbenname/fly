#pragma once
// PX4实际运动反馈上的水平外环：只形成姿态和偏航计划，不拥有推力、解锁或高度权限。
// 世界系为PX4 local NED（m/m/s），机体系为FRD；实际速度与制导速度必须分别校验。
// 外层单线程收包并保存原始源戳/接收时刻；本对象不建立ROS连接，不给缓存消息续鲜。
#include "jikl_fc01_system/vision_navigation.hpp"
#include <optional>

namespace fc01::horizontal {
using Vec3 = vision::Vec3;
using Quaternion = vision::Quaternion;
struct Actual {
  // 实测位置/速度为PX4局部NED：x北、y东、z下（m，m/s）；不把它们当任务目标。
  // 即使水平外环只用XY，z/vz仍须有限且有效；原生高度环在另一条既有链路独立检查它们。
  Vec3 position{vision::missing_array<3>()}, velocity{vision::missing_array<3>()};
  // Hamilton wxyz四元数把机体FRD向量旋转到PX4局部NED。
  Quaternion q{vision::missing_array<4>()};
  bool valid{};
  // 原始PX4采样时间已转换到主机ROS时间域（ns），接收时刻使用本机单调秒，不能互相替代。
  std::uint64_t local_stamp_ns{}, attitude_stamp_ns{};
  double local_received_at{-1}, attitude_received_at{-1};
  // source区分实际DDS写者，origin区分PX4局部参考/重置代；空值不允许凭猜测补齐。
  std::string source_id, origin_id;
  std::uint8_t xy_reset{}, vxy_reset{}, z_reset{}, vz_reset{}, quat_reset{};
};
struct Velocity {
  // sequence属于制导生产者会话；相同序号只能是完全重复报文，且不得更新received_at续鲜。
  std::uint64_t sequence{}, stamp_ns{};
  double received_at{-1};
  std::uint32_t reset_counter{};
  std::string source_id, frame_id, origin_id;
  // command stamp是制导生成时刻；attitude_sample_stamp_ns是编码向量用的原始PX4姿态采样时刻。
  std::uint64_t attitude_sample_stamp_ns{};
  double matched_attitude_received_at{-1};
  bool valid{}, matched_attitude_valid{};
  // FRD期望速度（m/s）：前/右/下；一般倾斜姿态的水平世界意图可能带body-z补偿。
  Vec3 body_velocity{vision::missing_array<3>()};
  // 新封套约定的NED航向变化率（rad/s），正为顺时针；不是PX4实测机体角速度。
  double yaw_rate{vision::missing()};
  Quaternion matched_attitude{vision::missing_array<4>()};
  std::uint64_t matched_attitude_stamp_ns{};
};
struct Input {
  Actual actual;
  vision::Command navigation;
  Velocity velocity;
  std::uint64_t ros_now_ns{}, revocation_generation{};
  std::size_t navigation_publishers{}, velocity_publishers{};
};
// PX4 position-mode mission output. A single XY interpretation applies per cycle.
enum class PositionIntentKind { Waypoint, Velocity, Brake };
struct PositionIntent {
  PositionIntentKind kind;
  std::array<double, 2> xy{};  // NED m for Waypoint, NED m/s otherwise.
  double yaw{};
  bool operator==(const PositionIntent& other) const {
    return kind == other.kind && xy == other.xy && yaw == other.yaw;
  }
};
struct PositionMissionPolicy {
  Vec3 takeoff_ned;
  bool waypoint_enabled{};
  const std::string& waypoint_origin;
  double north{}, east{};
};
inline bool same(const Actual& a, const Actual& b) {
  return a.valid == b.valid && a.source_id == b.source_id && a.origin_id == b.origin_id &&
         a.local_stamp_ns == b.local_stamp_ns && a.attitude_stamp_ns == b.attitude_stamp_ns &&
         a.local_received_at == b.local_received_at &&
         a.attitude_received_at == b.attitude_received_at && a.xy_reset == b.xy_reset &&
         a.vxy_reset == b.vxy_reset && a.z_reset == b.z_reset && a.vz_reset == b.vz_reset &&
         a.quat_reset == b.quat_reset && vision::detail::same(a.position, b.position) &&
         vision::detail::same(a.velocity, b.velocity) && vision::detail::same(a.q, b.q);
}
// 连续性只比较线上报文本身；匹配姿态是本机随后补齐的派生证据，不是发布者改写报文。
// received_at仍须相同，保证重复同一sequence不能给旧命令重新计时。
inline bool same_wire(const Velocity& a, const Velocity& b) {
  return a.sequence == b.sequence && a.stamp_ns == b.stamp_ns && a.received_at == b.received_at &&
         a.reset_counter == b.reset_counter && a.source_id == b.source_id &&
         a.frame_id == b.frame_id && a.origin_id == b.origin_id &&
         a.attitude_sample_stamp_ns == b.attitude_sample_stamp_ns && a.valid == b.valid &&
         vision::detail::same(a.body_velocity, b.body_velocity) &&
         vision::detail::same(a.yaw_rate, b.yaw_rate);
}
// 最终发布检查则必须连派生证据一起相同：已计算的向量不能临近发布时换用另一姿态。
inline bool same(const Velocity& a, const Velocity& b) {
  return same_wire(a, b) && a.matched_attitude_received_at == b.matched_attitude_received_at &&
         a.matched_attitude_valid == b.matched_attitude_valid &&
         a.matched_attitude_stamp_ns == b.matched_attitude_stamp_ns &&
         vision::detail::same(a.matched_attitude, b.matched_attitude);
}
// Hamilton wxyz表示FRD→NED的主动旋转。完整三维矩阵后取XY，不能只取yaw或先扔掉body-z。
inline Vec3 body_to_ned(const Quaternion& value, const Vec3& v) {
  const auto normalized = attitude::normalized(value);
  if (!normalized || !vision::detail::finite(v))
    return vision::missing_array<3>();
  const auto q = *normalized;
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {
      (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
      2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
      2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2]};
}
class Controller {
public:
  explicit Controller(vision::Config config = {}) : c_(config) {
    c_.validate();
  }
  // 状态执行顺序：有效实测→来源/重置连续性→双指令连续性→唯一来源→仲裁→限幅→姿态计划。
  // 首次输入等待时锁定XY；航点正常撤销时有限刹停；已跟踪后失锁/故障直接请求原生下降。
  // const Input&只借用本周期快照；返回Result是计划副本，不发布DDS、不触及解锁/电机。
  // 本对象保存上次输入、目标XY/yaw及故障锁存；所属Core重新准备时会重建它，不能空中自清故障。
  // fixed_height只用于结果的解释，绝不从航点z/视觉vz推导新高度；Core也不接收该结果的高度授权。
  vision::Result step(const Input& in, double now, double fixed_height, double px4_yaw, double dt,
                      bool motion_authorized = true, const PositionMissionPolicy* position = nullptr) {
    position_intent_.reset();
    if (stopped_ || failed_)
      return last_;
    if (!c_.enabled)
      return wait("HORIZONTAL_DISABLED");
    if (!std::isfinite(now) || now < 0 || !std::isfinite(dt) || dt <= 0 ||
        dt > std::min(c_.state_timeout, c_.command_timeout) || !std::isfinite(fixed_height) ||
        fixed_height < 0 || fixed_height > c_.max_height || !std::isfinite(px4_yaw))
      return reject("HORIZONTAL_CONTROL_SAMPLE_INVALID");
    const auto actual_error = actual_valid(in, now);
    if (!actual_error.empty())
      return reject(actual_error);
    if (accepted_) {
      if (in.revocation_generation != accepted_input_.revocation_generation)
        return reject("HORIZONTAL_INPUT_REVOKED");
      if (now < last_now_ || in.ros_now_ns < accepted_input_.ros_now_ns)
        return reject("HORIZONTAL_CLOCK_REGRESSION");
      const auto& old = accepted_input_.actual;
      const auto& a = in.actual;
      if (a.source_id != old.source_id || a.origin_id != old.origin_id ||
          a.xy_reset != old.xy_reset || a.vxy_reset != old.vxy_reset || a.z_reset != old.z_reset ||
          a.vz_reset != old.vz_reset || a.quat_reset != old.quat_reset)
        return reject("HORIZONTAL_PX4_REFERENCE_CHANGED");
      if (a.local_stamp_ns < old.local_stamp_ns || a.attitude_stamp_ns < old.attitude_stamp_ns ||
          (a.local_stamp_ns == old.local_stamp_ns &&
           (!vision::detail::same(a.position, old.position) ||
            !vision::detail::same(a.velocity, old.velocity))) ||
          (a.attitude_stamp_ns == old.attitude_stamp_ns && !vision::detail::same(a.q, old.q)))
        return reject("HORIZONTAL_PX4_SAMPLE_REGRESSED_OR_CHANGED");
      if (vision::detail::distance(a.position, old.position) > c_.max_position_jump ||
          vision::detail::distance(a.velocity, old.velocity) > c_.max_velocity_jump)
        return reject("HORIZONTAL_PX4_ESTIMATOR_JUMP");
    } else {
      // 半径中心在本任务首次获准使用水平外环的当前XY捕获，通常已定高；不是原点(0,0)，
      // 也不冒充地面起飞点。后续reset/source异常只退出，不在空中重新捕获半径中心。
      origin_ = position_target_ = position ? position->takeoff_ned : in.actual.position;
      if (position)
        waypoint_reference_ = origin_;
      yaw_target_ = vision::detail::wrap(px4_yaw);
    }
    // Freshness alone cannot authorize intents emitted before this task reached stable height.
    // Discard all envelopes already visible on the first authorized tick and require later stamps.
    if (position && motion_authorized && !position_authorized_stamp_) {
      position_authorized_stamp_ = in.ros_now_ns;
      position_navigation_floor_ = in.navigation.sequence;
      position_velocity_floor_ = in.velocity.sequence;
    }
    if (std::hypot(in.actual.position[0] - origin_[0], in.actual.position[1] - origin_[1]) >
        c_.max_radius)
      return reject("HORIZONTAL_RADIUS_EXCEEDED");
    const bool current_velocity_epoch = !position || (in.velocity.stamp_ns > position_authorized_stamp_ &&
        in.velocity.sequence > position_velocity_floor_);
    const bool current_waypoint_epoch = !position || (in.navigation.stamp_ns > position_authorized_stamp_ &&
        in.navigation.sequence > position_navigation_floor_);
    // 独立检查两个流的连续性。撤销也推进水位，不能让同周期旧有效消息复活。
    if ((current_waypoint_epoch && !observe_navigation(in.navigation)) ||
        (current_velocity_epoch && !observe_velocity(in.velocity)))
      return reject("HORIZONTAL_COMMAND_IDENTITY_OR_SEQUENCE_CHANGED");
    if (in.navigation_publishers > 1 || in.velocity_publishers > 1)
      return reject("HORIZONTAL_COMMAND_SOURCE_NOT_UNIQUE");
    const bool tracking = current_velocity_epoch && velocity_valid(in, now);
    const bool waypoint = current_waypoint_epoch && navigation_valid(in, now);
    // 跟踪已开始后必须维持本路可靠输入；旧航点不能掩盖失锁/撤销/过期。
    if (tracking_seen_ && !tracking)
      return land("HORIZONTAL_TRACK_LOST_OR_WITHDRAWN");
    // Once the waypoint is withdrawn, it cannot take over from a late/replayed command
    // while the independent vision stream is arriving.
    if (position && moved_ && transition_since_ >= 0 && waypoint && !tracking)
      return land("POSITION_WAYPOINT_REAPPEARED_AFTER_WITHDRAWAL");
    // 若有效声明了跟踪而载荷失效，不能让另一路航点掩盖故障；正常valid=false只撤销该路。
    // DDS不同话题的到达顺序并不保证一致。首次跟踪前，编码姿态尚在途时只等待，
    // 仍检查线上请求的源戳/原点/数值；不能把NaN或错原点也当作“等姿态”。已跟踪后不放宽。
    const bool waiting_for_attitude =
        !tracking_seen_ && !in.velocity.matched_attitude_valid && velocity_wire_valid(in, now) &&
        vision::detail::clock_valid(in.velocity.attitude_sample_stamp_ns, in.velocity.received_at,
                                    in.ros_now_ns, now, c_.state_timeout, c_);
    if (current_velocity_epoch && in.velocity.valid && !tracking && !waiting_for_attitude)
      return reject("HORIZONTAL_TRACK_INPUT_INVALID");
    // 未选中的旧NAV帧不覆盖当前可靠跟踪；它的来源/序号异常仍由上面的独立连续性门拒绝。
    if (current_waypoint_epoch && !tracking && in.navigation.valid && !waypoint)
      return reject("HORIZONTAL_WAYPOINT_INPUT_INVALID");
    if (position && moved_ && !motion_authorized)
      return land("HORIZONTAL_HEIGHT_AUTHORITY_LOST");
    if (!motion_authorized) {
      // 原生高度稳定窗暂时撤销水平授权时，仍须逐周期消费来源、序号、时效和失锁证据。
      // 此处不积分偏航、不选择新XY目标、不建立首次跟踪资格，且返回active=false，
      // 因而Core只能保留原生垂向闭环与水平姿态；已跟踪的真实失锁已在上方请求下降。
      if (tracking || waypoint)
        transition_since_ = -1;
      else if (moved_) {
        if (transition_since_ < 0)
          transition_since_ = now;
        if (now - transition_since_ >= c_.command_timeout)
          return land("HORIZONTAL_SWITCH_TIMEOUT");
      }
      accepted_input_ = in;
      last_now_ = now;
      last_ = {};
      last_.target_height = fixed_height;
      last_.reason = "HORIZONTAL_HEIGHT_AUTHORITY_SUSPENDED";
      return last_;
    }
    vision::Result out;
    out.active = true;
    out.target_height = fixed_height;
    out.yaw = yaw_target_;
    out.state_sequence = in.actual.local_stamp_ns;
    out.reset_counter = in.velocity.reset_counter;
    // 跟踪优先；同周期两路有效仍只选一个，不能叠加速度或重复推进偏航。
    if (tracking) {
      out.mode = vision::Mode::Follow;
      out.reason = "HORIZONTAL_TRACK_PX4_VELOCITY_CLOSED_LOOP";
      out.command_sequence = in.velocity.sequence;
      out.velocity_target = body_to_ned(in.velocity.matched_attitude, in.velocity.body_velocity);
      // 仅body-z的请求是垂直意图，不具任何水平权限。一般三维向量必须保留其投影补偿项。
      if (in.velocity.body_velocity[0] == 0 && in.velocity.body_velocity[1] == 0)
        out.velocity_target = {0, 0, 0};
      out.velocity_target[2] = 0;
      yaw_target_ = vision::detail::wrap(
          yaw_target_ + std::clamp(in.velocity.yaw_rate, -c_.max_yaw_rate, c_.max_yaw_rate) * dt);
      moved_ = tracking_seen_ = true;
      transition_since_ = -1;
    } else if (waypoint) {
      out.mode = in.navigation.mode;
      out.reason = out.mode == vision::Mode::Hold ? "HORIZONTAL_POINT_HOLD" : "HORIZONTAL_WAYPOINT";
      out.command_sequence = in.navigation.sequence;
      if (out.mode == vision::Mode::Land)
        return position ? reject("POSITION_MISSION_UNSUPPORTED_NAVIGATION_COMMAND")
                        : land("HORIZONTAL_LAND_REQUESTED");
      if (out.mode == vision::Mode::Cruise) {
        if (position && (!position->waypoint_enabled ||
            position->waypoint_origin != in.actual.origin_id ||
            in.navigation.position[0] != position->north || in.navigation.position[1] != position->east))
          return reject("POSITION_WAYPOINT_NOT_CONFIGURED_FOR_THIS_ORIGIN");
        position_target_ = in.navigation.position;
        if (std::hypot(position_target_[0] - origin_[0], position_target_[1] - origin_[1]) >
            c_.max_radius)
          return reject("HORIZONTAL_WAYPOINT_OUTSIDE_RADIUS");
      } else if (!position && (!accepted_ || mode_ != vision::Mode::Hold)) {
        position_target_ = in.actual.position;
      }
      if (position && out.mode == vision::Mode::Hold && moved_)
        return land("POSITION_WAYPOINT_WITHDRAWN_TO_HOLD");
      out.velocity_target = {c_.position_gain * (position_target_[0] - in.actual.position[0]),
                             c_.position_gain * (position_target_[1] - in.actual.position[1]), 0};
      if (!position && in.navigation.yaw_valid)
        yaw_target_ = vision::detail::wrap(
            yaw_target_ + std::clamp(vision::detail::wrap(in.navigation.yaw - yaw_target_),
                                     -c_.max_yaw_rate * dt, c_.max_yaw_rate * dt));
      moved_ = moved_ || !position || out.mode == vision::Mode::Cruise;
      transition_since_ = -1;
    } else {
      // 首次取得可靠输入前只锁当前XY、保持原生定高；不会因无相机自行降落。
      // 航点撤销而下一路尚在途时刹停，最多等待一个命令TTL；跟踪后双撤销是故障退出。
      if (tracking_seen_)
        return land("HORIZONTAL_TRACK_LOST_OR_WITHDRAWN");
      out.mode = vision::Mode::Hold;
      out.reason = moved_ ? "HORIZONTAL_NORMAL_SWITCH_BRAKE" : "HORIZONTAL_WAIT_INPUT_POINT_HOLD";
      if (moved_) {
        if (transition_since_ < 0)
          transition_since_ = now;
        if (now - transition_since_ >= c_.command_timeout)
          return land("HORIZONTAL_SWITCH_TIMEOUT");
        position_target_ = in.actual.position;
        out.velocity_target = {0, 0, 0};
      } else {
        out.velocity_target = {c_.position_gain * (position_target_[0] - in.actual.position[0]),
                               c_.position_gain * (position_target_[1] - in.actual.position[1]), 0};
      }
    }
    if (!vision::detail::finite(out.velocity_target) ||
        !std::isfinite(std::hypot(out.velocity_target[0], out.velocity_target[1])))
      return reject("HORIZONTAL_NONFINITE_PLAN");
    vision::detail::bound_xy(out.velocity_target, c_.max_speed);
    if (position) {
      if (tracking) {
        // 保守地在半径边缘制动。不生成新的越界位置目标，实测位置仍有独立半径门。
        const double room = c_.max_radius - std::hypot(in.actual.position[0] - origin_[0],
                                                      in.actual.position[1] - origin_[1]);
        const double speed = std::min(c_.max_speed,
            std::sqrt(2. * c_.max_acceleration * std::max(0., room)));
        const double current_speed = std::hypot(in.actual.velocity[0], in.actual.velocity[1]);
        if (room <= 0 || (current_speed > speed &&
            ((in.actual.position[0] - origin_[0]) * in.actual.velocity[0] +
             (in.actual.position[1] - origin_[1]) * in.actual.velocity[1]) > 0))
          return land("POSITION_TRACK_INSUFFICIENT_STOPPING_DISTANCE");
        vision::detail::bound_xy(out.velocity_target, speed);
        auto delta = Vec3{out.velocity_target[0] - last_velocity_[0],
                          out.velocity_target[1] - last_velocity_[1], 0};
        vision::detail::bound_xy(delta, c_.max_acceleration * dt);
        last_velocity_ = {last_velocity_[0] + delta[0], last_velocity_[1] + delta[1], 0};
        vision::detail::bound_xy(last_velocity_, speed);
        position_intent_ = PositionIntent{PositionIntentKind::Velocity,
                                         {last_velocity_[0], last_velocity_[1]}, yaw_target_};
      } else if (waypoint && out.mode == vision::Mode::Cruise) {
        auto delta = Vec3{position_target_[0] - waypoint_reference_[0],
                          position_target_[1] - waypoint_reference_[1], 0};
        vision::detail::bound_xy(delta, c_.max_speed * dt);
        const double next_x = waypoint_reference_[0] + delta[0];
        const double next_y = waypoint_reference_[1] + delta[1];
        // 限制随动参考领先实际位置的距离，防止飞机暂时受阻时累计很大的位置阶跃。
        if (std::hypot(next_x - in.actual.position[0], next_y - in.actual.position[1]) <=
            c_.max_speed / c_.position_gain) {
          waypoint_reference_[0] = next_x;
          waypoint_reference_[1] = next_y;
        }
        position_intent_ = PositionIntent{PositionIntentKind::Waypoint,
                                         {waypoint_reference_[0], waypoint_reference_[1]}, yaw_target_};
      } else if (moved_) {
        position_intent_ = PositionIntent{PositionIntentKind::Brake, {0, 0}, yaw_target_};
      }
      out.yaw = yaw_target_;
      accepted_ = true;
      accepted_input_ = in;
      mode_ = out.mode;
      last_now_ = now;
      last_ = out;
      return last_;
    }
    out.acceleration_target = {c_.velocity_gain * (out.velocity_target[0] - in.actual.velocity[0]),
                               c_.velocity_gain * (out.velocity_target[1] - in.actual.velocity[1]),
                               0};
    if (!vision::detail::finite(out.acceleration_target) ||
        !std::isfinite(std::hypot(out.acceleration_target[0], out.acceleration_target[1])))
      return reject("HORIZONTAL_NONFINITE_PLAN");
    const double acceleration_limit =
        std::min(c_.max_acceleration,
                 vision::detail::gravity * std::tan(c_.max_tilt_deg * vision::detail::pi / 180));
    vision::detail::bound_xy(out.acceleration_target, acceleration_limit);
    out.yaw = yaw_target_;
    out.q = vision::detail::attitude_target(out.acceleration_target, yaw_target_);
    if (!attitude::normalized(out.q))
      return reject("HORIZONTAL_NONFINITE_ATTITUDE");
    accepted_ = true;
    accepted_input_ = in;
    mode_ = out.mode;
    last_now_ = now;
    last_ = out;
    return last_;
  }
  // 三次最终发布检查均使用同一份已计算计划的输入内容；不会在检查中重新积分速度/偏航。
  bool current(const Input& in, double now) const {
    return accepted_ && !failed_ && !stopped_ && last_.active && std::isfinite(now) &&
           now >= last_now_ && in.ros_now_ns >= accepted_input_.ros_now_ns &&
           actual_valid(in, now).empty() &&
           in.revocation_generation == accepted_input_.revocation_generation &&
           in.navigation_publishers <= 1 && in.velocity_publishers <= 1 &&
           same(in.actual, accepted_input_.actual) &&
           // 只读当前被选中的指令。首次定点等待期间，新任务发布者出现或另一路新包
           // 不会让本周期的定点计划变危险；下一周期再仲裁。已选中指令的撤销仍立即失效。
           (last_.mode != vision::Mode::Follow ||
            (same(in.velocity, accepted_input_.velocity) && velocity_valid(in, now))) &&
           ((last_.mode != vision::Mode::Cruise && last_.reason != "HORIZONTAL_POINT_HOLD") ||
            (vision::detail::same(in.navigation, accepted_input_.navigation) &&
             navigation_valid(in, now))) &&
           (transition_since_ < 0 || now - transition_since_ < c_.command_timeout);
  }
  const vision::Result& last() const {
    return last_;
  }
  bool started() const {
    return accepted_;
  }
  const std::optional<PositionIntent>& position_intent() const { return position_intent_; }
  bool moved() const { return moved_; }
  void stop() {
    stopped_ = true;
    position_intent_.reset();
    last_ = {};
    last_.reason = "HORIZONTAL_STOPPED";
  }

private:
  std::string actual_valid(const Input& in, double now) const {
    const auto& a = in.actual;
    if (!a.valid || a.source_id.empty() || a.origin_id.empty() ||
        !vision::identity_within_budget(a.source_id) ||
        !vision::identity_within_budget(a.origin_id) || !vision::detail::finite(a.position) ||
        !vision::detail::finite(a.velocity) || !attitude::normalized(a.q))
      return "HORIZONTAL_PX4_ACTUAL_INVALID";
    if (!vision::detail::clock_valid(a.local_stamp_ns, a.local_received_at, in.ros_now_ns, now,
                                     c_.state_timeout, c_) ||
        !vision::detail::clock_valid(a.attitude_stamp_ns, a.attitude_received_at, in.ros_now_ns,
                                     now, c_.state_timeout, c_))
      return "HORIZONTAL_PX4_ACTUAL_STALE";
    return {};
  }
  bool navigation_valid(const Input& in, double now) const {
    const auto& n = in.navigation;
    return in.navigation_publishers == 1 && n.valid && n.sequence && !n.source_id.empty() &&
           vision::identities_within_budget(n) &&
           n.frame_id == "px4_local_ned:" + in.actual.origin_id &&
           vision::detail::clock_valid(n.stamp_ns, n.received_at, in.ros_now_ns, now,
                                       c_.command_timeout, c_) &&
           (n.mode == vision::Mode::Hold || n.mode == vision::Mode::Cruise ||
            n.mode == vision::Mode::Land) &&
           (n.mode != vision::Mode::Cruise ||
            (n.position_valid && std::isfinite(n.position[0]) && std::isfinite(n.position[1]))) &&
           (!n.yaw_valid || std::isfinite(n.yaw));
  }
  bool velocity_wire_valid(const Input& in, double now) const {
    const auto& v = in.velocity;
    return in.velocity_publishers == 1 && v.valid && v.sequence && !v.source_id.empty() &&
           vision::identity_within_budget(v.source_id) && v.frame_id == "body_frd" &&
           vision::detail::clock_valid(v.stamp_ns, v.received_at, in.ros_now_ns, now,
                                       c_.command_timeout, c_) &&
           v.attitude_sample_stamp_ns && v.origin_id == in.actual.origin_id &&
           vision::detail::finite(v.body_velocity) && std::isfinite(v.yaw_rate);
  }
  bool velocity_valid(const Input& in, double now) const {
    const auto& v = in.velocity;
    return velocity_wire_valid(in, now) && v.matched_attitude_valid &&
           v.matched_attitude_stamp_ns && v.attitude_sample_stamp_ns &&
           v.attitude_sample_stamp_ns == v.matched_attitude_stamp_ns &&
           vision::detail::clock_valid(v.attitude_sample_stamp_ns, v.matched_attitude_received_at,
                                       in.ros_now_ns, now, c_.state_timeout, c_) &&
           attitude::normalized(v.matched_attitude).has_value();
  }
  bool observe_navigation(const vision::Command& n) {
    // 任务启动后、平台原点尚未到达时可发送valid=false的未初始化撤销报文。
    // 首个有效意图之前不把这些空身份锁为生产者会话；有效意图出现后撤销也必须严格连续。
    if (!nav_seen_ && !n.valid)
      return true;
    if (!n.sequence)
      return true;
    if (nav_seen_ &&
        (n.source_id != previous_nav_.source_id || n.reset_counter != previous_nav_.reset_counter ||
         n.sequence < previous_nav_.sequence || n.stamp_ns < previous_nav_.stamp_ns ||
         (n.sequence == previous_nav_.sequence && !vision::detail::same(n, previous_nav_))))
      return false;
    nav_seen_ = true;
    previous_nav_ = n;
    return true;
  }
  bool observe_velocity(const Velocity& v) {
    if (!vel_seen_ && !v.valid)
      return true;
    if (!v.sequence)
      return true;
    if (vel_seen_ &&
        (v.source_id != previous_velocity_.source_id ||
         v.reset_counter != previous_velocity_.reset_counter ||
         v.sequence < previous_velocity_.sequence || v.stamp_ns < previous_velocity_.stamp_ns ||
         (v.sequence == previous_velocity_.sequence && !same_wire(v, previous_velocity_))))
      return false;
    vel_seen_ = true;
    previous_velocity_ = v;
    return true;
  }
  vision::Result wait(const std::string& reason) {
    last_ = {};
    last_.reason = reason;
    return last_;
  }
  vision::Result reject(const std::string& reason) {
    return moved_ || tracking_seen_ ? land(reason) : wait(reason);
  }
  vision::Result land(const std::string& reason) {
    failed_ = true;
    last_ = {};
    last_.request_land = true;
    last_.mode = vision::Mode::Land;
    last_.reason = reason;
    return last_;
  }
  vision::Config c_;
  Input accepted_input_;
  vision::Command previous_nav_;
  Velocity previous_velocity_;
  vision::Result last_;
  Vec3 origin_{}, position_target_{};
  Vec3 waypoint_reference_{}, last_velocity_{};
  std::optional<PositionIntent> position_intent_;
  double yaw_target_{}, last_now_{}, transition_since_{-1};
  std::uint64_t position_authorized_stamp_{}, position_navigation_floor_{}, position_velocity_floor_{};
  bool accepted_{}, moved_{}, tracking_seen_{}, failed_{}, stopped_{}, nav_seen_{}, vel_seen_{};
  vision::Mode mode_{vision::Mode::Disabled};
};
} // namespace fc01::horizontal
