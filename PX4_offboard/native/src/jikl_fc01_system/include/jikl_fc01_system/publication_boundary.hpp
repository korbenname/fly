#pragma once
// 最终发布包装器：把 Core 输出在真正调用 ROS sink 前重新进行纯策略检查。
// 目标和心跳不是原子事务：目标已提交后，任何心跳拒绝都属于部分发布，必须显式抛出异常。
// Context 回调每次取最新反馈和时钟；PublicationAttempt 保留走到哪一步及本地提交事实。
#include "jikl_fc01_system/publication_guard.hpp"
#include "jikl_fc01_system/attitude_wire.hpp"
namespace fc01::attitude {
// 本地sink证据与任务故障分开：策略拒绝不是发布者异常，发出target也不等于完整发出一对消息。
// None=未尝试，InitialContext=初查，BeforeTarget/BeforeHeartbeat=各 sink 前，Complete=两者返回。
enum class PublicationStage { None, InitialContext, BeforeTarget, BeforeHeartbeat, Complete };
inline const char* name(PublicationStage stage) {
  switch (stage) {
  case PublicationStage::None:
    return "NOT_ATTEMPTED";
  case PublicationStage::InitialContext:
    return "INITIAL_CONTEXT";
  case PublicationStage::BeforeTarget:
    return "BEFORE_TARGET";
  case PublicationStage::BeforeHeartbeat:
    return "BEFORE_HEARTBEAT";
  case PublicationStage::Complete:
    return "COMPLETE";
  }
  return "UNKNOWN";
}
// decision 是最近一次判定；committed 只表示本地 sink 返回，sink_exception 单独标注异常。
struct PublicationAttempt {
  // 初始化时stage为None，三个事实位全false；每次尝试从新证据开始，不拼上次发布的半包。
  PublicationStage stage{PublicationStage::None};
  Decision decision{};
  bool target_committed{}, heartbeat_committed{}, sink_exception{};
};
// 解锁请求也必须经过最终上下文策略，不能因为 Core 已计划就无条件调用 sink。
template <class Sink> Decision publish_arm_at_boundary(const BoundaryContext& c, Sink&& sink) {
  // Sink&&为模板转发引用，可借用本次lambda或函数对象；这里只同步调用，不保存到下一控制周期。
  const auto decision = arm_publication_policy(c);
  if (decision.allowed)
    sink();
  return decision;
}
// 上锁要求原有新地面证据，重复事务不能依靠未关联错误 ACK 自动重发。
template <class Sink> Decision publish_disarm_at_boundary(const BoundaryContext& c, Sink&& sink) {
  const auto decision = disarm_publication_policy(c);
  if (decision.allowed)
    sink();
  return decision;
}
// 释放 Offboard 仍只允许原策略证明的地面条件，不能当成飞行中切模式捷径。
template <class Sink> Decision publish_release_at_boundary(const BoundaryContext& c, Sink&& sink) {
  const auto decision = release_publication_policy(c);
  if (decision.allowed)
    sink();
  return decision;
}
// PX4 降落后备命令只有其合同及当前定位证明完整时才允许。
template <class Sink>
Decision publish_land_fallback_at_boundary(const BoundaryContext& c, Sink&& sink) {
  const auto decision = land_fallback_publication_policy(c);
  if (decision.allowed)
    sink();
  return decision;
}
// 策略/编码失败时两个sink均不可达；target sink抛异常就不发心跳，原异常交上层故障/退出流程。
// 静态上下文版本供确定性调用；编码失败时两个 sink 都不可达，目标 sink 异常时不发心跳。
template <class Mode, class Target, class TargetSink, class ModeSink>
Decision publish_attitude_at_boundary(const BoundaryContext& c, const Output& output,
                                      std::uint64_t stamp, TargetSink&& target_sink,
                                      ModeSink&& mode_sink) {
  auto decision = attitude_publication_policy(c, output);
  if (!decision.allowed)
    return decision;
  auto packet = wire<Mode, Target>(output, stamp);
  if (!packet)
    return deny(BoundaryReason::InvalidOutput);
  if (!nonzero(output.thrust))
    packet->second.thrust_body.fill(0.f);
  target_sink(packet->second);
  mode_sink(packet->first);
  return allow();
}
// 生产使用的重采样版本：初查→编码→目标前复查→目标提交→心跳前复查→心跳提交。
template <class Mode, class Target, class Context, class TargetSink, class ModeSink>
Decision publish_attitude_resampled(Context&& context, const Output& output, std::uint64_t stamp,
                                    TargetSink&& target_sink, ModeSink&& mode_sink,
                                    PublicationAttempt* evidence = nullptr) {
  // context是“现在重新取快照”的可调用对象，不是缓存的BoundaryContext；三次调用可能得到不同反馈。
  // evidence为可空的借用指针：调用者提供时原位填证据，未提供时用局部local，绝不保存悬空地址。
  PublicationAttempt local;
  auto& attempt = evidence ? *evidence : local;
  // auto&给被选中的同一个对象取别名，不复制；attempt={}清本次记录，不能撤销此前已发ROS消息。
  attempt = {};
  const auto check = [&](PublicationStage stage) {
    // [&]让lambda同步借用attempt/context/output；离开本函数前完成所有调用，不返回这个lambda。
    attempt.stage = stage;
    attempt.decision = attitude_publication_policy(context(), output);
    return attempt.decision;
  };
  auto decision = check(PublicationStage::InitialContext);
  if (!decision.allowed)
    return decision;
  auto packet = wire<Mode, Target>(output, stamp);
  if (!packet) {
    attempt.decision = deny(BoundaryReason::InvalidOutput);
    return attempt.decision;
  }
  if (!nonzero(output.thrust))
    // 明确写float精确零，避免准备流中出现转换后的微小推力；非零控制仍由原策略完整检查。
    packet->second.thrust_body.fill(0.f);
  // 编码也会消耗任务输入剩余寿命；此处尚无sink运行，拒绝后原受控下降仍有机会重新计算。
  // 编码也消耗 TTL；尚未发任何消息时拒绝，仍可由原 native 下降流程处置。
  decision = check(PublicationStage::BeforeTarget);
  if (!decision.allowed)
    return decision;
  try {
    target_sink(packet->second);
    // 只有sink正常返回才置committed；表示本地publish调用完成，仍不是PX4已收到/采用。
    attempt.target_committed = true;
  } catch (...) {
    attempt.sink_exception = true;
    // catch(...)捕获任意异常类型；裸throw原样重新抛出，不伪造成功，也不吞掉最初错误。
    throw;
  }
  // target已提交后再拒绝（含输入TTL到期）就是部分发布，不能返回“本轮完全没发”。
  // 此时目标已发；不能静默返回“未发”，否则上层会误判为完整未执行。
  decision = check(PublicationStage::BeforeHeartbeat);
  if (!decision.allowed)
    throw std::runtime_error(std::string("HEARTBEAT_FINAL_CONTEXT_CHANGED_") +
                             name(decision.reason));
  try {
    mode_sink(packet->first);
    attempt.heartbeat_committed = true;
  } catch (...) {
    attempt.sink_exception = true;
    throw;
  }
  attempt.stage = PublicationStage::Complete;
  // Complete只证明两个本地sink依次返回；最终动作/Runtime回记和后续PX4状态仍是其他层证据。
  return allow();
}

// 位置模式独占轨迹出口。地面准入只发 position 心跳，等 PX4 真正进入 Offboard 再送轨迹。
enum class PositionDemand { HeartbeatOnly, GroundIdle, ClimbOrHold, Descend };

inline PositionDemand position_demand(const BoundaryContext& c, const Output& output) {
  if (rc_stage(c.phase) || !c.feedback.armed || !c.feedback.offboard)
    return PositionDemand::HeartbeatOnly;
  switch (c.phase) {
  case Phase::ThrustRamp:
    // ArmedGround 可在同一步进入 ThrustRamp，该步尚未推进高度轨迹。
    // 继续地面抑制目标，避免有限的 z0 因估计噪声被解释为上升请求。
    if (output.target_height == 0.)
      return PositionDemand::GroundIdle;
    [[fallthrough]];
  case Phase::Ascend:
  case Phase::HeightHold:
    return PositionDemand::ClimbOrHold;
  case Phase::Descent:
  case Phase::RecoveryDescent:
  case Phase::Px4LandPending:
    return PositionDemand::Descend;
  default:
    return PositionDemand::GroundIdle;
  }
}

inline Decision position_publication_policy(const BoundaryContext& c, const Output& output) {
  if (!c.config.px4_takeoff_position_hold)
    return deny(BoundaryReason::WrongProfile);
  if (nonzero(output.thrust) || output.visual_motion || output.command == Command::Arm ||
      output.command == Command::Offboard)
    return deny(BoundaryReason::InvalidOutput);
  if (!rc_stage(c.phase) && !c.feedback.offboard)
    return deny(BoundaryReason::ArmedOffboard);
  const auto demand = position_demand(c, output);
  if (output.position_intent) {
    const auto& intent = *output.position_intent;
    if (!c.config.px4_position_mission_enabled || c.phase != Phase::HeightHold ||
        !c.liftoff_confirmed || c.feedback.landed || !c.visual_motion_allowed ||
        !c.position_intent || !(*c.position_intent == intent) ||
        output.target_height != c.config.rc_target_height)
      return deny(BoundaryReason::VisualInput);
    if (!std::isfinite(intent.yaw) || !vision::detail::finite(intent.xy))
      return deny(BoundaryReason::InvalidOutput);
    const auto& anchor = c.takeoff_position_anchor;
    switch (intent.kind) {
    case horizontal::PositionIntentKind::Waypoint:
      if (!c.config.px4_position_waypoint_enabled ||
          c.config.px4_position_waypoint_origin_id != c.feedback.horizontal_input.actual.origin_id ||
          std::hypot(intent.xy[0] - anchor.x, intent.xy[1] - anchor.y) > c.config.visual.max_radius)
        return deny(BoundaryReason::VisualInput);
      break;
    case horizontal::PositionIntentKind::Velocity:
    case horizontal::PositionIntentKind::Brake:
      if (std::hypot(intent.xy[0], intent.xy[1]) > c.config.visual.max_speed + 1e-9)
        return deny(BoundaryReason::VisualInput);
      break;
    default:
      return deny(BoundaryReason::InvalidOutput);
    }
  }
  const bool moving = demand == PositionDemand::ClimbOrHold || demand == PositionDemand::Descend;
  // 下降线路发向下速度，起降同高轨迹字段此时不会进入报文。轻微超调后仍须能正常降落。
  if (demand == PositionDemand::ClimbOrHold &&
      (!std::isfinite(output.target_height) || output.target_height < 0 ||
       output.target_height > c.config.rc_target_height))
    return deny(BoundaryReason::FlightEnvelope);
  return setpoint_publication_policy(c, output, moving);
}

template <class Target>
Target position_target(const BoundaryContext& c, PositionDemand demand, const Output& output,
                       std::uint64_t stamp) {
  Target target{};
  target.timestamp = stamp;
  target.position.fill(NAN);
  target.velocity.fill(NAN);
  target.acceleration.fill(NAN);
  target.jerk.fill(NAN);
  target.yaw = float(c.yaw_reference);
  target.yawspeed = NAN;
  // 各轴至少有一类有限目标。离地证明之前，零水平加速度不请求 XY 位置或速度闭环。
  const bool hold_xy = c.liftoff_confirmed && !c.feedback.landed &&
                       (demand == PositionDemand::ClimbOrHold || demand == PositionDemand::Descend);
  if (hold_xy) {
    auto xy = std::array<double, 2>{c.takeoff_position_anchor.x, c.takeoff_position_anchor.y};
    if (demand == PositionDemand::Descend && c.position_descent_hold)
      xy = *c.position_descent_hold;
    target.position[0] = float(xy[0]);
    target.position[1] = float(xy[1]);
  } else {
    target.acceleration[0] = target.acceleration[1] = 0.f;
  }
  if (demand == PositionDemand::ClimbOrHold)
    target.position[2] = float(c.reference_z - output.target_height);
  else if (demand == PositionDemand::Descend)
    // NED 向下为正。PX4 需要明确的下行速度目标，才能触发其地面接触检测。
    target.velocity[2] = float(c.config.px4_takeoff_position_descent_speed);
  else
    target.acceleration[2] = 9.80665f;
  if (output.position_intent) {
    const auto& intent = *output.position_intent;
    target.yaw = float(intent.yaw);
    for (std::size_t axis = 0; axis < intent.xy.size(); ++axis) {
      if (intent.kind == horizontal::PositionIntentKind::Waypoint)
        target.position[axis] = float(intent.xy[axis]);
      else {
        target.position[axis] = NAN;
        target.velocity[axis] = float(intent.xy[axis]);
      }
    }
  }
  return target;
}

template <class Mode, class Target, class Context, class TargetSink, class ModeSink>
Decision publish_position_resampled(Context&& context, const Output& output, std::uint64_t stamp,
                                   TargetSink&& target_sink, ModeSink&& mode_sink,
                                   PublicationAttempt& attempt) {
  attempt = {};
  const auto check = [&](PublicationStage stage, const BoundaryContext& current) {
    attempt.stage = stage;
    attempt.decision = position_publication_policy(current, output);
    return attempt.decision;
  };
  const auto initial = context();
  auto decision = check(PublicationStage::InitialContext, initial);
  if (!decision.allowed)
    return decision;
  const auto demand = position_demand(initial, output);
  const auto target = position_target<Target>(initial, demand, output, stamp);
  if (!stamp || !std::isfinite(target.yaw) ||
      (demand == PositionDemand::ClimbOrHold && !std::isfinite(target.position[2])))
    return attempt.decision = deny(BoundaryReason::InvalidOutput);
  const auto verify = [&](PublicationStage stage) {
    const auto current = context();
    const auto result = check(stage, current);
    if (!result.allowed)
      return result;
    // 重采样只能否决已编码的目标，禁止在心跳阶段把目标重新绑定到其他位置或阶段。
    if (position_demand(current, output) != demand || current.phase != initial.phase ||
        current.feedback.landed != initial.feedback.landed ||
        current.liftoff_confirmed != initial.liftoff_confirmed ||
        current.reference_z != initial.reference_z || current.yaw_reference != initial.yaw_reference ||
        current.position_descent_hold != initial.position_descent_hold ||
        !(current.takeoff_position_anchor.frame == initial.takeoff_position_anchor.frame) ||
        current.takeoff_position_anchor.x != initial.takeoff_position_anchor.x ||
        current.takeoff_position_anchor.y != initial.takeoff_position_anchor.y)
      return attempt.decision = deny(BoundaryReason::EstimatorReset);
    return result;
  };
  if (demand != PositionDemand::HeartbeatOnly) {
    decision = verify(PublicationStage::BeforeTarget);
    if (!decision.allowed)
      return decision;
    try {
      target_sink(target);
      attempt.target_committed = true;
    } catch (...) {
      attempt.sink_exception = true;
      throw;
    }
  }
  decision = verify(PublicationStage::BeforeHeartbeat);
  if (!decision.allowed) {
    if (attempt.target_committed)
      throw std::runtime_error(std::string("HEARTBEAT_FINAL_CONTEXT_CHANGED_") + name(decision.reason));
    return decision;
  }
  Mode mode{};
  mode.timestamp = stamp;
  mode.position = true;
  try {
    mode_sink(mode);
    attempt.heartbeat_committed = true;
  } catch (...) {
    attempt.sink_exception = true;
    throw;
  }
  attempt.stage = PublicationStage::Complete;
  return allow();
}
} // namespace fc01::attitude
