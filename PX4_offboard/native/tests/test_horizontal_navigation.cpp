// 纯水平外环反例与方向回放；真实PX4运动反馈和制导请求分离，无ROS/硬件动作。
#include "jikl_fc01_system/horizontal_navigation.hpp"
#include <stdexcept>
#include <iostream>
namespace h = fc01::horizontal;
namespace v = fc01::vision;
namespace a = fc01::attitude;
static void need(bool b, const char* m) {
  if (!b)
    throw std::runtime_error(m);
}
static bool near(double a, double b, double eps = 1e-9) {
  return std::abs(a - b) < eps;
}
static v::Config config() {
  v::Config c;
  c.enabled = true;
  return c;
}
static h::Input input(double now = 10) {
  h::Input in;
  in.ros_now_ns = std::uint64_t(now * 1e9);
  auto& p = in.actual;
  p.valid = true;
  p.source_id = "PX4-source";
  p.origin_id = "local:1";
  p.local_stamp_ns = p.attitude_stamp_ns = in.ros_now_ns;
  p.local_received_at = p.attitude_received_at = now;
  p.position = {0, 0, .5};
  p.velocity = {0, 0, 0};
  p.q = {1, 0, 0, 0};
  in.navigation_publishers = in.velocity_publishers = 1;
  auto& n = in.navigation;
  n.sequence = 1;
  n.stamp_ns = in.ros_now_ns;
  n.received_at = now;
  n.source_id = "mission";
  n.frame_id = "px4_local_ned:" + p.origin_id;
  n.mode = v::Mode::Hold;
  auto& s = in.velocity;
  s.sequence = 1;
  s.stamp_ns = in.ros_now_ns;
  s.received_at = now;
  s.source_id = "mission";
  s.frame_id = "body_frd";
  s.origin_id = p.origin_id;
  return in;
}
static void advance(h::Input& in, double now) {
  in.ros_now_ns = std::uint64_t(now * 1e9);
  auto& p = in.actual;
  p.local_stamp_ns = p.attitude_stamp_ns = in.ros_now_ns;
  p.local_received_at = p.attitude_received_at = now;
  for (auto* s : {&in.navigation}) {
    ++s->sequence;
    s->stamp_ns = in.ros_now_ns;
    s->received_at = now;
  }
  auto& s = in.velocity;
  ++s.sequence;
  s.stamp_ns = in.ros_now_ns;
  s.received_at = now;
  s.attitude_sample_stamp_ns = s.matched_attitude_stamp_ns = in.ros_now_ns;
  s.matched_attitude_received_at = now;
}
static void velocity(h::Input& in, h::Vec3 body, double wz = 0) {
  auto& x = in.velocity;
  x.valid = x.matched_attitude_valid = true;
  x.body_velocity = body;
  x.yaw_rate = wz;
  x.attitude_sample_stamp_ns = x.matched_attitude_stamp_ns = in.actual.attitude_stamp_ns;
  x.matched_attitude = in.actual.q;
  x.matched_attitude_received_at = in.actual.attitude_received_at;
}
static void math_and_yaw() {
  auto in = input();
  const double p = 30 * v::detail::pi / 180;
  in.actual.q = {std::cos(p / 2), 0, std::sin(p / 2), 0};
  // 世界北0.4m/s在俯仰机体中有body-z补偿项；完整旋转必须恢复0.4而不是0.3。
  velocity(in, {.4 * std::cos(p), 0, .4 * std::sin(p)});
  h::Controller c(config());
  auto o = c.step(in, 10, 1.5, 0, .05);
  need(o.active && near(o.velocity_target[0], .4) && near(o.velocity_target[1], 0),
       "full3D projection");
  need(near(o.target_height, 1.5) && o.velocity_target[2] == 0, "no vertical authority");
  need(v::detail::yaw(o.q) == 0 && o.q[2] < 0, "north acceleration pitches forward");
  auto yaw90 = input();
  yaw90.actual.q = {std::cos(v::detail::pi / 4), 0, 0, std::sin(v::detail::pi / 4)};
  velocity(yaw90, {.4, 0, 0});
  auto east = h::Controller(config()).step(yaw90, 10, 1.5, v::detail::pi / 2, .05);
  need(east.active && near(east.velocity_target[0], 0) && near(east.velocity_target[1], .4),
       "FRDforward with eastheading maps to NEDeast");
  auto roll30 = input();
  roll30.actual.q = {std::cos(p / 2), std::sin(p / 2), 0, 0};
  velocity(roll30, {0, .4 * std::cos(p), -.4 * std::sin(p)});
  auto rolled = h::Controller(config()).step(roll30, 10, 1.5, 0, .05);
  need(rolled.active && near(rolled.velocity_target[0], 0) && near(rolled.velocity_target[1], .4),
       "full3D rollprojection preserves horizontal right intent");
  auto only = input();
  only.actual.q = in.actual.q;
  velocity(only, {0, 0, .5});
  auto po = h::Controller(config()).step(only, 10, 1.5, 0, .05);
  need(po.active && po.velocity_target[0] == 0 && po.velocity_target[1] == 0,
       "bodyvertical has no XY authority");
  for (double right : {-.3, .3}) {
    auto q = input();
    velocity(q, {0, right, 0});
    auto r = h::Controller(config()).step(q, 10, 1.5, 0, .05);
    need(r.active && r.q[1] * right > 0, "right left tilt sign");
  }
  auto q = input();
  velocity(q, {0, 0, 0}, .3);
  h::Controller yaw(config());
  auto first = yaw.step(q, 10, 1.5, v::detail::pi - .005, .05);
  need(first.active && first.yaw < 0, "positive NEDyaw wraps pi clockwise");
  advance(q, 10.05);
  velocity(q, {0, 0, 0}, 0);
  auto zero = yaw.step(q, 10.05, 1.5, -1., .05);
  need(near(zero.yaw, first.yaw), "wz0 holds command yaw not actualyaw");
  advance(q, 10.1);
  velocity(q, {0, 0, 0}, -.3);
  auto negative = yaw.step(q, 10.1, 1.5, -1., .05);
  need(near(v::detail::wrap(negative.yaw - zero.yaw), -.015), "negativewz sign");
}
static void arbitration_and_faults() {
  auto in = input();
  in.navigation.valid = true;
  in.navigation.mode = v::Mode::Cruise;
  in.navigation.position_valid = true;
  in.navigation.position = {.5, 0, 9999};
  velocity(in, {0, .2, 0}, .1);
  h::Controller c(config());
  auto o = c.step(in, 10, 1.5, 0, .05);
  need(o.active && o.mode == v::Mode::Follow && o.velocity_target[0] == 0 &&
           o.velocity_target[1] > .1,
       "one winner tracking not sum");
  auto next = in;
  advance(next, 10.05);
  next.velocity.valid = false;
  need(c.step(next, 10.05, 1.5, 0, .05).request_land, "old waypoint cannot hide tracking loss");
  auto nav = input();
  nav.navigation.valid = true;
  nav.navigation.mode = v::Mode::Cruise;
  nav.navigation.position_valid = true;
  nav.navigation.position = {.5, 0, -10000};
  h::Controller switcher(config());
  auto p = switcher.step(nav, 10, 1.5, 0, .05);
  need(p.active && p.velocity_target[0] > .1 && p.target_height == 1.5, "waypointXY only ignoreZ");
  auto no_vertical_number = input();
  no_vertical_number.navigation = nav.navigation;
  no_vertical_number.navigation.position[2] = NAN;
  auto unchanged_height = h::Controller(config()).step(no_vertical_number, 10, 1.5, 0, .05);
  need(unchanged_height.active && unchanged_height.target_height == 1.5,
       "irrelevant waypointz never enters height arithmetic");
  advance(nav, 10.05);
  nav.navigation.valid = false;
  auto brake = switcher.step(nav, 10.05, 1.5, 0, .05);
  need(brake.active && !brake.request_land && brake.velocity_target[0] == 0,
       "normal navwithdraw finite brake");
  advance(nav, 10.10);
  velocity(nav, {0, .2, 0});
  need(switcher.step(nav, 10.10, 1.5, 0, .05).mode == v::Mode::Follow,
       "crosscyclewithdraw newtrack");
  for (int kind = 0; kind < 8; ++kind) {
    auto x = input();
    velocity(x, {.2, 0, 0});
    h::Controller fault(config());
    need(fault.step(x, 10, 1.5, 0, .05).active, "faultstart");
    advance(x, 10.05);
    velocity(x, {.2, 0, 0});
    if (kind == 0)
      x.actual.valid = false;
    if (kind == 1)
      ++x.actual.xy_reset;
    if (kind == 2)
      x.actual.source_id = "other";
    if (kind == 3)
      x.velocity.source_id = "other";
    if (kind == 4)
      x.velocity.body_velocity[0] = NAN;
    if (kind == 5)
      x.velocity_publishers = 2;
    if (kind == 6)
      x.velocity.origin_id = "other";
    if (kind == 7)
      ++x.revocation_generation;
    need(fault.step(x, 10.05, 1.5, 0, .05).request_land, "tracking fault must descend");
  }
}
static void boundary_and_bounds() {
  auto x = input();
  velocity(x, {100, 100, 100}, 100);
  h::Controller c(config());
  auto o = c.step(x, 10, 1.5, 0, .05);
  need(o.active && std::hypot(o.velocity_target[0], o.velocity_target[1]) <= .500001,
       "speed bound");
  need(std::hypot(o.acceleration_target[0], o.acceleration_target[1]) <= .600001,
       "acceleration bound");
  need(std::abs(o.yaw) <= .350001 * .05, "yaw bound");
  auto overspeed = input();
  overspeed.actual.velocity = {.4, 0, 0};
  velocity(overspeed, {.2, 0, 0});
  auto brake = h::Controller(config()).step(overspeed, 10, 1.5, 0, .05);
  need(brake.active && brake.acceleration_target[0] < 0 && brake.q[2] > 0,
       "PX4 actualvelocity exceeding demand produces opposite brake tilt");
  need(c.current(x, 10.01), "exact current");
  auto changed = x;
  changed.actual.position[0] = .1;
  need(!c.current(changed, 10.01), "lastboundaryactual change");
  changed = x;
  changed.velocity.valid = false;
  need(!c.current(changed, 10.01), "lastboundarywithdraw");
  changed = x;
  changed.ros_now_ns += 400000000;
  need(!c.current(changed, 10.4), "oldsource cannot renew");
  auto m = input();
  velocity(m, {.2, 0, 0});
  m.velocity.matched_attitude_stamp_ns -= 1;
  need(!h::Controller(config()).step(m, 10, 1.5, 0, .05).active, "EXACT paired attitude required");
  auto nav = input();
  nav.navigation.valid = true;
  nav.navigation.mode = v::Mode::Cruise;
  nav.navigation.position_valid = true;
  nav.navigation.position = {2.1, 0, 0};
  need(!h::Controller(config()).step(nav, 10, 1.5, 0, .05).active, "radius bound");
}
static void callback_order_and_selected_dependency() {
  auto bootstrap = input();
  bootstrap.navigation.source_id = bootstrap.velocity.source_id = "mission-before-platform";
  bootstrap.navigation.frame_id.clear();
  bootstrap.velocity.origin_id.clear();
  h::Controller late_mission(config());
  need(late_mission.step(bootstrap, 10, 1.5, 0, .05).active,
       "initial uninitialized withdrawn channels onlywait");
  advance(bootstrap, 10.05);
  bootstrap.navigation.source_id = bootstrap.velocity.source_id = "mission";
  bootstrap.navigation.frame_id = "px4_local_ned:" + bootstrap.actual.origin_id;
  bootstrap.navigation.reset_counter = bootstrap.velocity.reset_counter = 1;
  bootstrap.velocity.origin_id = bootstrap.actual.origin_id;
  velocity(bootstrap, {.2, 0, 0});
  need(late_mission.step(bootstrap, 10.05, 1.5, 0, .05).mode == v::Mode::Follow,
       "first reliable intent can initialize after withdrawn bootstrap");
  // 定点等待时任务节点才出现：未选中意图的到达不能使旧定点计划触发下降。
  auto initial = input();
  initial.navigation_publishers = initial.velocity_publishers = 0;
  initial.navigation = {};
  initial.velocity = {};
  h::Controller waiting(config());
  need(waiting.step(initial, 10, 1.5, 0, .05).active, "native point hold before mission exists");
  auto arrived = initial;
  arrived.navigation_publishers = arrived.velocity_publishers = 1;
  arrived.navigation = input().navigation;
  need(waiting.current(arrived, 10.01), "new mission graph does not revoke initialhold");
  arrived.velocity_publishers = 2;
  need(!waiting.current(arrived, 10.01), "duplicate source still revokes initialhold");

  // 命令早于原始姿态回调抵达，稍后补齐同一采样姿态：线上sequence不变，不得误判篡改。
  auto pending = input();
  velocity(pending, {.2, 0, 0});
  pending.velocity.matched_attitude_valid = false;
  pending.velocity.matched_attitude_stamp_ns = 0;
  pending.velocity.matched_attitude_received_at = -1;
  pending.velocity.matched_attitude = v::missing_array<4>();
  h::Controller paired(config());
  auto hold = paired.step(pending, 10, 1.5, 0, .05);
  need(hold.active && hold.mode == v::Mode::Hold, "wait original attitude without assuming freshq");
  auto enriched = pending;
  enriched.velocity.matched_attitude_valid = true;
  enriched.velocity.matched_attitude_stamp_ns = pending.actual.attitude_stamp_ns;
  enriched.velocity.matched_attitude_received_at = pending.actual.attitude_received_at;
  enriched.velocity.matched_attitude = pending.actual.q;
  need(paired.current(enriched, 10.01), "enrichment does not invalidate independent hold");
  auto track = paired.step(enriched, 10.02, 1.5, 0, .02);
  need(track.active && track.mode == v::Mode::Follow, "samewire sequence enriched to tracking");
  auto inactive_nav = enriched;
  ++inactive_nav.navigation.sequence;
  inactive_nav.navigation.valid = false;
  need(paired.current(inactive_nav, 10.03), "inactive navwithdraw cannot mask reliabletracking");
  auto changed = enriched;
  changed.velocity.body_velocity[0] = .3;
  need(paired.step(changed, 10.04, 1.5, 0, .02).request_land,
       "samewire changed vector still faults");

  // 普通切换的刹停期限在最终发布边界也要重查，不能靠计算开始时未超期续发。
  auto nav = input();
  nav.navigation.valid = true;
  h::Controller transition(config());
  need(transition.step(nav, 10, 1.5, 0, .05).active, "navhold beforewithdraw");
  advance(nav, 10.05);
  nav.navigation.valid = false;
  need(transition.step(nav, 10.05, 1.5, 0, .05).active, "transitionbrake start");
  // 用较长状态TTL隔离验证切换期限本身，避免把状态过期误当作本项通过的原因。
  auto long_ttl = config();
  long_ttl.state_timeout = 1.;
  h::Controller expiry(long_ttl);
  nav = input();
  nav.navigation.valid = true;
  need(expiry.step(nav, 10, 1.5, 0, .05).active, "expiry navstart");
  advance(nav, 10.05);
  nav.navigation.valid = false;
  need(expiry.step(nav, 10.05, 1.5, 0, .05).active, "expiry braked");
  nav.ros_now_ns = std::uint64_t(10.36 * 1e9);
  need(!expiry.current(nav, 10.36), "finalboundary checks boundedbrake timeout");
}
static void suspended_height_authority() {
  auto in = input();
  velocity(in, {.2, 0, 0}, .3);
  h::Controller controller(config());
  const auto first = controller.step(in, 10, 1.5, 0, .05);
  need(first.active, "suspend starttracking");
  advance(in, 10.05);
  velocity(in, {.2, 0, 0}, .3);
  auto suspended = controller.step(in, 10.05, 1.5, 0, .05, false);
  need(!suspended.active && !suspended.request_land && !controller.current(in, 10.05),
       "validheight suspension no horizontal authority");
  advance(in, 10.1);
  velocity(in, {.2, 0, 0}, 0);
  const auto resumed = controller.step(in, 10.1, 1.5, 0, .05);
  need(resumed.active && near(resumed.yaw, first.yaw),
       "suspendedyaw rate was not integrated for later replay");
  advance(in, 10.15);
  in.velocity.valid = false;
  need(controller.step(in, 10.15, 1.5, 0, .05, false).request_land,
       "trackingloss despite noheightauthority immediately requestsdescent");
}
static void position_mission_authority() {
  auto in = input();
  in.actual.position = {2.25, -1.75, .5};
  const std::string origin = in.actual.origin_id;
  const h::PositionMissionPolicy policy{{2.25, -1.75, 2.}, true, origin, 2.65, -1.75};
  auto& nav = in.navigation;
  nav.valid = nav.position_valid = true;
  nav.mode = v::Mode::Cruise;
  nav.position = {2.65, -1.75, 20.};
  h::Controller controller(config());
  controller.step(in, 10., 1.5, 0., .05, true, &policy);
  need(!controller.position_intent(), "cached preheight waypoint must not move the target");
  advance(in, 10.05);
  nav.position[1] = -1.65;
  controller.step(in, 10.05, 1.5, 0., .05, true, &policy);
  need(!controller.position_intent(), "native rejects coordinates unlike the configured waypoint");
  advance(in, 10.10);
  nav.position[1] = -1.75;
  controller.step(in, 10.10, 1.5, 0., .05, true, &policy);
  const auto waypoint = controller.position_intent();
  need(waypoint && waypoint->kind == h::PositionIntentKind::Waypoint &&
           near(waypoint->xy[0], 2.275) && near(waypoint->xy[1], -1.75),
       "waypoint trajectory must start at the true ground anchor and have a bounded step");
  advance(in, 10.15);
  nav.valid = false;
  controller.step(in, 10.15, 1.5, 0., .05, true, &policy);
  need(controller.position_intent() &&
           controller.position_intent()->kind == h::PositionIntentKind::Brake,
       "withdrawn waypoint must wait with a brake command for the other stream");
  advance(in, 10.20);
  nav.valid = true;
  need(controller.step(in, 10.20, 1.5, 0., .05, true, &policy).request_land,
       "withdrawn waypoint cannot resume the old motion");
}

int main() {
  math_and_yaw();
  arbitration_and_faults();
  boundary_and_bounds();
  callback_order_and_selected_dependency();
  suspended_height_authority();
  position_mission_authority();
  std::cout << "HORIZONTAL_NAVIGATION_PASS\n";
}
