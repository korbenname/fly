// 完整Runtime→Core→最终发布边界的水平候选；RC仍人工，原生高度固定1.5m。
#include "vision_test_support.hpp"
#include <iostream>
using namespace vision_test;
namespace h = fc01::horizontal;
static void fill(VisionFlight& q, bool valid = false) {
  const double now = q.t + .05;
  const auto ns = std::uint64_t(std::llround(now * 1e9));
  auto& in = q.s.attitude.horizontal_input;
  in.ros_now_ns = ns;
  auto& p = in.actual;
  p.valid = true;
  p.position = {0, 0, .5};
  p.velocity = {0, 0, 0};
  p.q = {1, 0, 0, 0};
  p.source_id = "PX4-source";
  p.origin_id = "local:1";
  p.local_stamp_ns = p.attitude_stamp_ns = ns;
  p.local_received_at = p.attitude_received_at = now;
  in.navigation_publishers = in.velocity_publishers = 1;
  auto& n = in.navigation;
  ++n.sequence;
  n.stamp_ns = ns;
  n.received_at = now;
  n.source_id = "mission";
  n.frame_id = "px4_local_ned:" + p.origin_id;
  n.mode = v::Mode::Cruise;
  n.valid = valid;
  n.position_valid = true;
  n.position = {.5, 0, -9999};
  auto& vel = in.velocity;
  ++vel.sequence;
  vel.stamp_ns = ns;
  vel.received_at = now;
  vel.source_id = "mission";
  vel.frame_id = "body_frd";
  vel.origin_id = p.origin_id;
  vel.valid = false;
}
static void scenario(bool height_leaves_window) {
  auto c = config();
  c.attitude.px4_horizontal_enabled = true;
  VisionFlight q(c);
  fill(q, true);
  q.join();
  need(!q.o.attitude.visual_motion, "no horizontal beforeliftoff");
  for (int i = 0; i < 100 && q.r.attitude_core().phase() != a::Phase::ThrustRamp; ++i) {
    fill(q, true);
    q.tick();
  }
  double ht = 0;
  while (ht < 1.5 - 1e-9) {
    ht = std::min(1.5, ht + .01);
    q.observe(ht, .2);
    q.s.feedback.landed = q.s.at_rest = false;
    fill(q, true);
    q.tick();
    need(!q.o.attitude.visual_motion, "no horizontal duringclimb");
  }
  q.observe(1.5);
  for (int i = 0; i < 160 && q.r.attitude_core().phase() != a::Phase::HeightHold; ++i) {
    fill(q, true);
    q.tick();
  }
  fill(q, true);
  q.tick();
  need(q.r.attitude_core().navigation_allowed() && q.o.attitude.visual_motion &&
           q.last_decision.allowed,
       "horizontal afteractualsettled");
  need(q.last_target.q_d[2] < 0 && q.last_target.thrust_body[2] < 0, "actualtiltandnative thrust");
  need(q.r.attitude_core().target_height() == 1.5, "waypointz cannot takeoverheight");
  auto changed = q.s;
  changed.attitude.horizontal_input.actual.valid = false;
  auto context = q.r.publication_context(changed, q.t, true, true);
  need(!a::attitude_publication_policy(context, q.o.attitude).allowed,
       "finalactualinvalid blocksoldtilt");
  fill(q, false);
  q.tick();
  need(q.o.attitude.visual_motion && q.r.attitude_core().phase() == a::Phase::HeightHold,
       "normalwithdraw brake preservesheight");
  fill(q, false);
  auto& vel = q.s.attitude.horizontal_input.velocity;
  vel.valid = vel.matched_attitude_valid = true;
  vel.body_velocity = {0, .2, 0};
  vel.yaw_rate = .1;
  vel.matched_attitude = {1, 0, 0, 0};
  vel.attitude_sample_stamp_ns = vel.matched_attitude_stamp_ns = vel.stamp_ns;
  vel.matched_attitude_received_at = vel.received_at;
  q.tick();
  need(q.o.attitude.visual_motion && q.last_target.q_d[1] > 0, "switchtrack correctrighttilt");
  // 重走同样真实准备/爬升路径后分支：高度离开稳定窗与失锁同时到达，导航门关闭也须读故障。
  if (height_leaves_window)
    q.observe(1.5 + 2 * c.attitude.height_tolerance);
  fill(q, false);
  if (height_leaves_window)
    ++q.s.attitude.horizontal_input.revocation_generation;
  q.tick();
  need(q.r.attitude_core().phase() == a::Phase::Descent && q.o.attitude.publish &&
           !q.o.attitude.visual_motion,
       height_leaves_window ? "tracking fault whileheightunauthorized descends immediately"
                            : "trackloss controlleddescent not powercut");
  need(q.r.attitude_core().target_height() == 1.5, "failuredoesnotreplacefinaltarget");
}
int main() {
  scenario(false);
  scenario(true);
  std::cout << "HORIZONTAL_RUNTIME_PASS\n";
}
