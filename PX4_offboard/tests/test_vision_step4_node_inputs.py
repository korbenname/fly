"""真实SystemNode接收缓存/代数回归，ROS API替身、确定性时钟；不连接DDS或设备。"""
from pathlib import Path
import importlib
import os
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
HARNESS=r'''
#include <cstdint>
double recovery_test_steady=10.;
std::int64_t recovery_test_wall_ns=10000000000LL;
#include "adapter_under_test.cpp"
#include <cassert>
int main(){
 SystemNode n;n.config_.attitude.visual.enabled=true;n.config_.attitude.px4_horizontal_enabled=true;
 n.local_.timestamp_sample=10000000;n.horizontal_attitude_gid_="attitude-source";
 Attitude a{};a.timestamp_sample=10000000;a.q={1,0,0,0};n.attitude_sample_.received=10.;
 n.remember_horizontal_attitude(a);auto count=n.horizontal_attitude_index_;
 n.attitude_sample_.received=10.1;n.remember_horizontal_attitude(a);
 assert(n.horizontal_attitude_index_==count&&n.horizontal_attitudes_[0].received_at==10.);
 a.q={.99f,0,0,.1f};n.remember_horizontal_attitude(a);
 assert(n.horizontal_revocation_==1&&n.horizontal_attitudes_[0].q[0]==1.);
 std::cout<<"RAW_ATTITUDE_FIRST_CONTENT_AND_TTL_PASS\n";
 VisualCommand nav{};nav.sequence=1;nav.stamp.sec=10;nav.valid=false;
 n.receive_horizontal_navigation(nav);nav.sequence=2;nav.source_id="mission-new";
 nav.frame_id="px4_local_ned:"+n.horizontal_origin();nav.reset_counter=1;
 n.receive_horizontal_navigation(nav);assert(n.horizontal_revocation_==1);
 nav.sequence=3;nav.valid=true;nav.mode=1;n.receive_horizontal_navigation(nav);
 auto before=n.horizontal_revocation_;nav.sequence=4;nav.valid=false;n.receive_horizontal_navigation(nav);
 assert(n.horizontal_revocation_==before);
 std::cout<<"PREINTENT_BOOTSTRAP_AND_NORMAL_NAV_WITHDRAW_PASS\n";
 HorizontalCommand h{};h.command.sequence=1;h.command.stamp.sec=10;
 h.command.source_id="mission";h.command.frame_id="body_frd";h.origin_id=n.horizontal_origin();
 h.attitude_sample_stamp_ns=10000000000ULL;h.command.valid=true;n.receive_horizontal_velocity(h);
 before=n.horizontal_revocation_;h.command.sequence=2;h.command.valid=false;n.receive_horizontal_velocity(h);
 assert(n.horizontal_revocation_==before+1);h.command.sequence=3;n.receive_horizontal_velocity(h);
 assert(n.horizontal_revocation_==before+1);
 h.command.sequence=4;h.command.valid=true;n.receive_horizontal_velocity(h);
 assert(n.horizontal_revocation_==before+1);
 std::cout<<"TRACK_WITHDRAW_BURST_LATCH_WITHOUT_INVALID_RENEWAL_PASS\n";
 auto original=h.origin_id;before=n.horizontal_revocation_;
 h.command.sequence=5;h.origin_id="wrong";n.receive_horizontal_velocity(h);
 h.command.sequence=6;h.origin_id=original;n.receive_horizontal_velocity(h);
 assert(n.horizontal_revocation_==before+2);
 std::cout<<"COMMAND_ORIGIN_A_B_A_REVOKED_PASS\n";
 rclcpp::MessageInfo info;std::string writer;
 info.value.publisher_gid.data[0]=1;n.observe_horizontal_publisher(writer,info);
 before=n.horizontal_revocation_;info.value.publisher_gid.data[0]=2;n.observe_horizontal_publisher(writer,info);
 info.value.publisher_gid.data[0]=1;n.observe_horizontal_publisher(writer,info);
 assert(n.horizontal_revocation_==before+2);
 std::cout<<"TRANSPORT_GID_A_B_A_REVOKED_PASS\n";
 n.local_={};n.local_stamp_={};n.local_sample_={};
 Local loc{};loc.timestamp=loc.timestamp_sample=10000000;loc.xy_valid=loc.v_xy_valid=true;
 loc.z_valid=loc.v_z_valid=loc.heading_good_for_control=true;
 n.receive_sampled(loc,n.local_,n.local_stamp_,n.local_sample_,fc01::SourceSlot::Local,info);
 before=n.horizontal_revocation_;loc.timestamp+=1;loc.x=.1f;
 n.receive_sampled(loc,n.local_,n.local_stamp_,n.local_sample_,fc01::SourceSlot::Local,info);
 ++loc.timestamp;loc.x=0;
 n.receive_sampled(loc,n.local_,n.local_stamp_,n.local_sample_,fc01::SourceSlot::Local,info);
 assert(n.horizontal_revocation_==before+2);
 before=n.horizontal_revocation_;++loc.timestamp;++loc.timestamp_sample;loc.ref_timestamp=1;
 n.receive_sampled(loc,n.local_,n.local_stamp_,n.local_sample_,fc01::SourceSlot::Local,info);
 ++loc.timestamp;++loc.timestamp_sample;loc.ref_timestamp=0;
 n.receive_sampled(loc,n.local_,n.local_stamp_,n.local_sample_,fc01::SourceSlot::Local,info);
 assert(n.horizontal_revocation_==before+2);
 std::cout<<"RAW_LOCAL_MUTATION_AND_REFERENCE_A_B_A_REVOKED_PASS\n";
}
'''

class VisionStep4NodeInputs(unittest.TestCase):
    def test_actual_node_callback_and_history_edges(self):
        sys.path.insert(0,str(ROOT/'native/tests'))
        self.addCleanup(lambda:sys.path.remove(str(ROOT/'native/tests')))
        recovery=importlib.import_module('test_rc_recovery_adapter')
        evidence=Path('/tmp/fc01-step4-node-inputs');evidence.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='fc01-step4-inputs-') as folder:
            build=Path(folder);recovery.write_fixture(build)
            driver=build/'inputs.cpp';driver.write_text(HARNESS)
            binary=build/'inputs'
            compiled=subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-pthread',
                '-I',str(build),'-I',str(ROOT/'native/src/jikl_fc01_system/include'),str(driver),'-o',str(binary)],
                capture_output=True,text=True,timeout=120)
            (evidence/'compile.log').write_text(compiled.stdout+compiled.stderr)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            result=subprocess.run([str(binary)],env=dict(os.environ,FC01_SOCKET=str(build/'control.sock')),
                capture_output=True,text=True,timeout=15)
            (evidence/'result.log').write_text(result.stdout+result.stderr)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertEqual(result.stdout.count('_PASS'),6)

if __name__=='__main__':unittest.main()
