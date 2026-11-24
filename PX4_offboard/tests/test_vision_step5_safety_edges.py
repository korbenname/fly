"""第5步独立负面验证：真实节点缓存、原生最终检查和历史授权的交叉条件。

C++回归直接编译生产头文件或实际SystemNode方法，ROS API是本机替身。
不连接DDS/串口/systemd，不把纯内存通过解释为PX4接收或实机飞行。
"""
from pathlib import Path
import hashlib
import importlib
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

NODE_HARNESS = r'''
#include <cstdint>
double recovery_test_steady=10.;
std::int64_t recovery_test_wall_ns=10000000000LL;
#include "adapter_under_test.cpp"
#include <cassert>
int main() {
  SystemNode node;
  node.config_.attitude.visual.enabled=true;
  node.config_.attitude.px4_horizontal_enabled=true;
  node.local_.timestamp_sample=10000000;
  VisualCommand nav{};
  nav.sequence=10;nav.stamp.sec=10;nav.source_id="mission";
  nav.frame_id="px4_local_ned:"+node.horizontal_origin();nav.valid=true;nav.mode=1;
  node.receive_horizontal_navigation(nav);
  auto nav_age=node.horizontal_navigation_.received_at;
  recovery_test_steady=10.1;
  node.receive_horizontal_navigation(nav);
  assert(node.horizontal_navigation_.received_at==nav_age);
  auto generation=node.horizontal_revocation_;
  nav.sequence=11;nav.valid=false;node.receive_horizontal_navigation(nav);
  nav.sequence=10;nav.valid=true;node.receive_horizontal_navigation(nav);
  nav.sequence=12;node.receive_horizontal_navigation(nav);
  assert(node.horizontal_revocation_>generation);
  std::cout<<"NAV_WITHDRAW_OUT_OF_ORDER_HEALED_PACKET_STILL_REVOKED_PASS\n";

  HorizontalCommand command{};
  command.command.sequence=10;command.command.stamp.sec=10;
  command.command.source_id="mission";command.command.frame_id="body_frd";
  command.command.valid=true;command.command.vx=.2;
  command.origin_id=node.horizontal_origin();command.attitude_sample_stamp_ns=10000000000ULL;
  node.receive_horizontal_velocity(command);
  auto velocity_age=node.horizontal_velocity_.received_at;
  generation=node.horizontal_revocation_;
  recovery_test_steady=10.2;node.receive_horizontal_velocity(command);
  assert(node.horizontal_velocity_.received_at==velocity_age);
  assert(node.horizontal_revocation_==generation);
  command.command.sequence=11;command.command.valid=false;
  node.receive_horizontal_velocity(command);
  command.command.sequence=10;command.command.valid=true;
  node.receive_horizontal_velocity(command);
  command.command.sequence=12;node.receive_horizontal_velocity(command);
  assert(node.horizontal_revocation_>generation);
  std::cout<<"VELOCITY_DUPLICATE_NO_TTL_WITHDRAW_OLD_GOOD_NEW_GOOD_LATCH_PASS\n";

  generation=node.horizontal_revocation_;
  command.command.sequence=13;command.command.vx=.1;
  node.receive_horizontal_velocity(command);
  const auto first_value=node.horizontal_velocity_.body_velocity[0];
  command.command.vx=.3;node.receive_horizontal_velocity(command);
  assert(node.horizontal_revocation_>generation);
  assert(node.horizontal_velocity_.body_velocity[0]==first_value);
  std::cout<<"ALTERED_SAME_SEQUENCE_PRESERVES_FIRST_PAYLOAD_PASS\n";
}
'''

CONTROLLER_HARNESS = r'''
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main existing_horizontal_regression_main
#include "test_horizontal_navigation.cpp"
#undef main
#pragma GCC diagnostic pop
int main() {
  for (bool tracking : {false,true}) {
    auto in=input();
    if(tracking)velocity(in,{.2,0,0},.2);
    else {in.navigation.valid=true;in.navigation.mode=v::Mode::Cruise;
          in.navigation.position_valid=true;in.navigation.position={.3,0,0};}
    h::Controller controller(config());
    const auto plan=controller.step(in,10,1.5,0,.05);
    need(plan.active&&controller.current(in,10),"valid original plan");
    for (int changed : {0,1}) {
      auto boundary=in;
      if(changed==0)boundary.navigation_publishers=2;
      else boundary.velocity_publishers=2;
      need(!controller.current(boundary,10),"second writer on either channel blocks final sink");
    }
    auto changed=in;
    changed.revocation_generation++;
    need(!controller.current(changed,10),"middle invalid survives healed final payload");
    need(!controller.current(in,10.301),"final check expires original source+receive TTL");
    need(near(controller.last().yaw,plan.yaw),"boundary checks cannot integrate yaw");
  }
  std::cout<<"FINAL_BOUNDARY_EITHER_WRITER_CONFLICT_TTL_REVOCATION_PASS\n";
  auto in=input();velocity(in,{.2,0,0},.3);
  h::Controller controller(config());
  need(controller.step(in,10,1.5,0,.05).active,"tracking starts");
  // 新PX4真实样本不能给不推进sequence的旧指令续鲜。
  in.ros_now_ns=10310000000ULL;
  in.actual.local_stamp_ns=in.actual.attitude_stamp_ns=in.ros_now_ns;
  in.actual.local_received_at=in.actual.attitude_received_at=10.31;
  const auto expired=controller.step(in,10.31,1.5,0,.05);
  need(expired.request_land&&!expired.active,"command timeout while actual fresh descends");
  advance(in,10.36);velocity(in,{.2,0,0},0);
  const auto healthy=controller.step(in,10.36,1.5,0,.05);
  need(healthy.request_land&&!healthy.active,"new healthy frame cannot clear fault");
  std::cout<<"ACTUAL_FRESH_OLD_COMMAND_EXPIRES_AND_CANNOT_SELF_RECOVER_PASS\n";
}
'''


class VisionStep5SafetyEdges(unittest.TestCase):
    def compile_and_run(self, source, fixture=False):
        evidence=Path(os.environ.get('FC01_VISION_STEP5_SAFETY_EVIDENCE','/tmp/fc01-step5-safety-edges'))
        evidence.mkdir(parents=True,exist_ok=True)
        label=self.id().rsplit('.',1)[-1]
        with tempfile.TemporaryDirectory(prefix='fc01-step5-edges-') as temp:
            build=Path(temp)
            if fixture:
                sys.path.insert(0,str(ROOT/'native/tests'))
                try:importlib.import_module('test_rc_recovery_adapter').write_fixture(build)
                finally:sys.path.remove(str(ROOT/'native/tests'))
            driver=build/'edge.cpp';driver.write_text(source)
            binary=build/'edge'
            argv=['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-pthread',
                  '-I',str(build),'-I',str(ROOT/'native/src/jikl_fc01_system/include'),
                  '-I',str(ROOT/'native/tests'),str(driver),'-o',str(binary)]
            compiled=subprocess.run(argv,capture_output=True,text=True,timeout=120)
            (evidence/(label+'.build.log')).write_text(compiled.stdout+compiled.stderr)
            self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
            run=subprocess.run([str(binary)],env=dict(os.environ,FC01_SOCKET=str(build/'control.sock')),
                               capture_output=True,text=True,timeout=30)
            (evidence/(label+'.log')).write_text(run.stdout+run.stderr)
            self.assertEqual(run.returncode,0,run.stdout+run.stderr)
            (evidence/(label+'.json')).write_text(json.dumps(dict(passed=True,
                source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                production_node_sha256=hashlib.sha256((ROOT/'native/src/jikl_fc01_system/src/fc01_system_node.cpp').read_bytes()).hexdigest(),
                output=run.stdout,environment='host strict C++ with in-memory ROS API fixtures; no hardware'),indent=2)+'\n')
            return run.stdout

    def test_actual_node_reordered_and_duplicate_commands_do_not_renew_or_clear_revocation(self):
        text=self.compile_and_run(NODE_HARNESS,True)
        self.assertEqual(text.count('_PASS'),3)

    def test_final_boundary_catches_writer_conflict_age_and_healed_payload(self):
        text=self.compile_and_run(CONTROLLER_HARNESS)
        self.assertEqual(text.count('_PASS'),2)


if __name__=='__main__':
    unittest.main()
