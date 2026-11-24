#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_attitude_boundary_tests
#include "test_publication_boundary.cpp"
#undef main
#pragma GCC diagnostic pop

struct PositionTarget {
  std::uint64_t timestamp{};
  std::array<float, 3> position{}, velocity{}, acceleration{}, jerk{};
  float yaw{}, yawspeed{};
};

int main() {
  auto c = context(true);
  c.phase = a::Phase::Descent;
  c.config.px4_takeoff_position_hold = c.config.rc_navigation_enabled = true;
  c.config.rc_navigation_mode_slot = 1;
  c.config.rc_takeover_confirmed = true;
  c.config.autonomous_failsafe_confirmed = c.config.arm_enabled = false;
  c.config.max_flight_seconds = 30;
  c.config.max_tilt_deg = 10;
  c.authority = {fc01::authority::Mode::RcTakeover, fc01::authority::Mode::RcTakeover, true, false};
  auto& f = c.feedback;
  f.armed = true;
  f.landed = false;
  f.rc_link_ready = f.rc_switches_ready = f.rc_source_selected = true;
  f.rc_source_clock_valid = f.rc_switch_clock_valid = f.rc_axes_safe = f.rc_aux_switches_safe = true;
  f.rc_switches_seen = f.rc_switches_fresh = true;
  f.rc_kill_switch = 3;
  f.takeoff_position = {1., 2., 0., 0., {12, 1}, true};
  c.takeoff_position_anchor = {1., 2., f.takeoff_position.frame, true};
  c.liftoff_confirmed = true;

  // 超调后 LAND 同周期仍可能携带高于 1.5 m 的旧轨迹高度。实际下降报文只使用 VZ。
  a::Output plan;
  plan.publish = true;
  plan.target_height = 1.56;
  assert(a::position_publication_policy(c, plan).allowed);
  a::PublicationAttempt evidence;
  bool target_written = false;
  bool heartbeat_written = false;
  const auto target_sink = [&](const PositionTarget& target) {
    assert(target.position[0] == 1.f && target.position[1] == 2.f);
    assert(std::isnan(target.position[2]) && target.velocity[2] == .30f);
    target_written = true;
  };
  const auto mode_sink = [&](const Mode& mode) {
    assert(target_written && mode.position && !mode.attitude);
    heartbeat_written = true;
  };
  const auto accepted = a::publish_position_resampled<Mode, PositionTarget>(
      [&] { return c; }, plan, 123, target_sink, mode_sink, evidence);
  assert(accepted.allowed && target_written && heartbeat_written);

  c.phase = a::Phase::Ascend;
  assert(!a::position_publication_policy(c, plan).allowed);
  plan.target_height = 1.5;
  c.rc_manual_arm_proven = true;
  f.rc_arm_switch = f.rc_mode_slot = 1;
  f.latest_arming_reason = 2;
  f.armed_time = c.rc_manual_armed_time = 11;
  assert(a::position_publication_policy(c, plan).allowed);
  unsigned calls = 0;
  target_written = heartbeat_written = false;
  bool refused_partial = false;
  try {
    a::publish_position_resampled<Mode, PositionTarget>(
        [&] {
          auto next = c;
          if (++calls == 3)
            ++next.feedback.takeoff_position.frame.xy_reset;
          return next;
        }, plan, 124, [&](const PositionTarget&) { target_written = true; }, mode_sink, evidence);
  } catch (const std::runtime_error&) {
    refused_partial = true;
  }
  assert(refused_partial && target_written && !heartbeat_written);
  assert(evidence.target_committed && !evidence.heartbeat_committed);
}
