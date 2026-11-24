// 第3步：真实Runtime/Core/最终双出口合成回归，不连接飞控或生成解锁命令。
#include "vision_test_support.hpp"
#include <iostream>
using namespace vision_test;

static void repeated_single_height_sample_cannot_complete_settling() {
  auto c = config();
  c.attitude.visual.enabled = false;
  c.attitude.settle_seconds = .2;
  c.attitude.local_timeout = .5;
  VisionFlight q(c);
  q.join();
  for (int i = 0; i < 100 && q.r.attitude_core().phase() != a::Phase::ThrustRamp; ++i)
    q.tick();
  double h = 0;
  while (h < c.attitude.rc_target_height - 1e-9) {
    h = std::min(c.attitude.rc_target_height, h + .01);
    q.observe(h, .2);
    q.s.feedback.landed = q.s.at_rest = false;
    q.tick();
  }
  q.observe(c.attitude.rc_target_height, 0);
  // 先让平滑目标轨迹到达最终目标，同时保留上升/垂速未稳定。
  q.observe(c.attitude.rc_target_height, .2);
  for (int i = 0; i < 30; ++i)
    q.tick();
  q.observe(c.attitude.rc_target_height, 0);
  q.tick();
  need(!q.r.attitude_core().navigation_allowed(), "one measured at-height sample is not settled");
  const auto local = q.s.attitude.local_stamp, raw = q.s.attitude.vertical_sample_stamp;
  for (int i = 0; i < 6; ++i) {
    q.t += .05;
    q.s.status_stamp += 50000;
    q.s.land_stamp += 50000;
    q.s.attitude.rc_switch_sample_stamp += 50000;
    q.s.attitude.source_host_stamp = q.s.status_stamp;
    q.s.attitude.local_stamp = local;
    q.s.attitude.vertical_sample_stamp = raw;
    q.o = q.r.step(q.s, q.t);
    if (q.o.attitude.publish) {
      const auto decision = a::publish_attitude_at_boundary<RcMode, RcTarget>(
          q.r.publication_context(q.s, q.t, true, true), q.o.attitude, q.s.status_stamp,
          [](const RcTarget&) {
          },
          [](const RcMode&) {
          });
      q.r.confirm_rc_output(decision.allowed, q.t);
    }
    need(!q.r.attitude_core().navigation_allowed(),
         "duplicate local sample manufactured continuous target-height evidence");
  }
}

static void measured_departure_revokes_horizontal_but_preserves_vertical_hold() {
  auto c = config();
  c.attitude.visual.enabled = false;
  VisionFlight q(c);
  q.reach_hold();
  need(q.r.attitude_core().navigation_allowed(), "new continuous settling admits navigation");
  q.observe(c.attitude.rc_target_height + .2, 0);
  q.tick();
  need(q.r.attitude_core().phase() == a::Phase::HeightHold,
       "height departure must retain native vertical control");
  need(!q.r.attitude_core().navigation_allowed(),
       "height outside tolerance must revoke horizontal authorization");
  need(q.o.attitude.publish && q.last_decision.allowed, "native vertical hold stays publishable");
  need(std::string(q.r.attitude_core().takeoff_stage()) == "TARGET_HEIGHT_SETTLING",
       "height departure must not erase established liftoff evidence in the displayed stage");
  q.observe(c.attitude.rc_target_height, 0);
  q.tick();
  need(!q.r.attitude_core().navigation_allowed(),
       "returning single sample cannot restore navigation");
  for (int i = 0; i < 10; ++i)
    q.tick();
  need(q.r.attitude_core().navigation_allowed(), "fresh continuous resettling restores navigation");
}

// 实际回调一轮可能有多条local：中间越界必须要求先step，不能只保留健康尾帧。
static void local_callback_boundary_preserves_an_out_of_tolerance_sample() {
  auto c = config();
  c.attitude.visual.enabled = false;
  VisionFlight q(c);
  q.reach_hold();
  const auto& core = q.r.attitude_core();
  need(core.local_requires_control_step(q.s.attitude.vertical_sample_stamp + 1000,
                                        core.reference_z() - c.attitude.rc_target_height - .2, 0,
                                        0),
       "brief height excursion after settling must cross the callback control boundary");
  need(core.local_requires_control_step(q.s.attitude.vertical_sample_stamp + 1000, q.s.attitude.z,
                                        c.attitude.speed_tolerance + .01, 0),
       "brief vertical speed excursion must cross the callback control boundary");
}

// 最后出口只能按新快照报告任务授权；Core上周期到高不能代替当前DDS/模式/reset检查。
static void final_snapshot_changes_revoke_navigation_without_rebasing_z0() {
  auto c = config();
  c.attitude.visual.enabled = false;
  VisionFlight q(c);
  q.reach_hold();
  const auto reference = q.r.attitude_core().reference_z();
  const auto current = [&](const f::Snapshot& s) {
    const auto context = q.r.publication_context(s, q.t, true, true);
    return q.r.attitude_core().navigation_current(context.feedback, q.t);
  };
  need(current(q.s), "current settled feedback is authorized");
  for (int kind = 0; kind < 7; ++kind) {
    auto changed = q.s;
    if (kind == 0)
      changed.local_fresh = false;
    if (kind == 1)
      changed.feedback.offboard = false;
    if (kind == 2)
      ++changed.attitude.z_reset;
    if (kind == 3)
      ++changed.attitude.vz_reset;
    if (kind == 4)
      ++changed.attitude.quat_reset;
    if (kind == 5)
      changed.attitude.z -= .2;
    if (kind == 6)
      changed.attitude.vz = c.attitude.speed_tolerance + .01;
    need(!current(changed), "new stale/mode/reset/height/speed snapshot must revoke navigation");
    need(q.r.attitude_core().reference_z() == reference,
         "current authorization query cannot rebase z0");
  }
  for (bool vertical_velocity_reset : {false, true}) {
    VisionFlight reset(c);
    reset.reach_hold();
    if (vertical_velocity_reset)
      ++reset.s.attitude.vz_reset;
    else
      ++reset.s.attitude.z_reset;
    reset.tick();
    need(!reset.r.attitude_core().navigation_allowed() && !reset.o.attitude.publish,
         "real reset enters existing protection instead of reusing old at-height state");
    need(reset.r.attitude_core().reference_z() == reference,
         "armed estimator reset cannot silently capture new ground");
  }
}

// 主机延迟拉长的短源序列、同一源戳换值和缓存超时都不能伪造连续稳定。
static void target_height_evidence_requires_independent_current_measurements() {
  auto c = config();
  c.attitude.settle_seconds = .2;
  c.attitude.local_timeout = .5;
  a::TargetHeightSettling compressed;
  for (int i = 0; i < 8; ++i)
    need(!compressed.observe(c.attitude, true, 1000000 + 1000 * i, 1.5, 0, .05 * i),
         "source window of milliseconds cannot become 200ms settling through host delay");
  a::TargetHeightSettling normal;
  for (int i = 0; i < 5; ++i)
    normal.observe(c.attitude, true, 1000000 + 50000 * i, 1.5, 0, .05 * i);
  need(normal.evidence().confirmed, "distinct source and host windows establish settling");
  need(!normal.observe(c.attitude, true, 1200000, 1.5, 0, .71) && !normal.evidence().confirmed,
       "expired repeated sample must revoke both return value and stored settled evidence");
  need(!normal.observe(c.attitude, true, 1200000, 1.5, 0, .72),
       "old sample cannot restart a fresh stable window");
  need(!normal.observe(c.attitude, true, 1250000, 1.5, 0, .76),
       "single new sample cannot resurrect old dwell");
  a::TargetHeightSettling changed;
  changed.observe(c.attitude, true, 1000000, 1.5, 0, 0);
  need(!changed.observe(c.attitude, true, 1000000, 1.51, 0, .05) && !changed.evidence().samples,
       "different values under one timestamp invalidate that source identity");
}
int main() {
  repeated_single_height_sample_cannot_complete_settling();
  measured_departure_revokes_horizontal_but_preserves_vertical_hold();
  local_callback_boundary_preserves_an_out_of_tolerance_sample();
  final_snapshot_changes_revoke_navigation_without_rebasing_z0();
  target_height_evidence_requires_independent_current_measurements();
  std::cout << "STAGE3_NATIVE_NAVIGATION_GATE_PASS\n";
}
