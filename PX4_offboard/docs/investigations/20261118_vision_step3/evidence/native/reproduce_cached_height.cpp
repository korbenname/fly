// 相同原始PX4位置样本在控制循环被重复读到，不能累计为新的到高测量。
// 编译时分别使用修复前HEAD与当前生产头文件；本程序不连接ROS或飞控。
#include "vision_test_support.hpp"
#include <iostream>
using namespace vision_test;
int main() {
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
  for (int i = 0; i < 30; ++i)
    q.tick();
  q.observe(c.attitude.rc_target_height, 0);
  q.tick();
  const auto local = q.s.attitude.local_stamp;
  const auto raw = q.s.attitude.vertical_sample_stamp;
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
    if (q.r.attitude_core().navigation_allowed()) {
      std::cerr << "REPRODUCED: one unchanged PX4 local sample manufactured target settling\n";
      return 1;
    }
  }
  std::cout << "PASS: duplicate local sample did not complete target settling\n";
}
