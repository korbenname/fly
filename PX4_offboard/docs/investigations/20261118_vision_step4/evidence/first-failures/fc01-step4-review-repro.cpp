#include "jikl_fc01_system/horizontal_navigation.hpp"
#include <iostream>
int main(){
 fc01::vision::Config c; c.enabled=true;
 fc01::horizontal::Controller h(c); fc01::horizontal::Input i;
 i.ros_now_ns=100000000000ULL;i.navigation_publishers=i.velocity_publishers=1;
 i.actual.valid=true; i.actual.position={0,0,-1.5};i.actual.velocity={0,0,0};i.actual.q={1,0,0,0};
 i.actual.source_id="px4";i.actual.origin_id="test";i.actual.local_stamp_ns=i.actual.attitude_stamp_ns=i.ros_now_ns;
 i.actual.local_received_at=i.actual.attitude_received_at=10.;
 i.navigation.sequence=1;i.navigation.stamp_ns=i.ros_now_ns;i.navigation.received_at=10.;i.navigation.source_id="mission";
 i.navigation.frame_id="px4_local_ned:test";i.navigation.valid=true;i.navigation.mode=fc01::vision::Mode::Cruise;
 i.navigation.position_valid=true;i.navigation.position={1,0,0};
 i.velocity.sequence=1;i.velocity.stamp_ns=i.ros_now_ns;i.velocity.received_at=10.;i.velocity.source_id="mission";
 i.velocity.frame_id="body_frd";i.velocity.valid=true;i.velocity.matched_attitude_valid=true;i.velocity.body_velocity={.2,0,0};
 i.velocity.yaw_rate=0;i.velocity.matched_attitude={1,0,0,0};i.velocity.matched_attitude_stamp_ns=i.ros_now_ns;i.velocity.attitude_sample_stamp_ns=i.ros_now_ns;i.velocity.origin_id="test";i.velocity.matched_attitude_received_at=i.actual.attitude_received_at;
 auto a=h.step(i,10,1.5,0,.02);std::cout<<"before="<<a.reason<<" land="<<a.request_land<<"\n";
 i.ros_now_ns+=20000000ULL;i.actual.local_stamp_ns=i.actual.attitude_stamp_ns=i.ros_now_ns;
 i.actual.local_received_at=i.actual.attitude_received_at=10.02;
 ++i.navigation.sequence;i.navigation.stamp_ns=i.ros_now_ns;i.navigation.received_at=10.02;
 ++i.velocity.sequence;i.velocity.stamp_ns=i.ros_now_ns;i.velocity.received_at=10.02;i.velocity.valid=false;
 i.velocity.matched_attitude_stamp_ns=i.ros_now_ns;i.velocity.attitude_sample_stamp_ns=i.ros_now_ns;i.velocity.origin_id="test";i.velocity.matched_attitude_received_at=i.actual.attitude_received_at;
 auto b=h.step(i,10.02,1.5,0,.02);std::cout<<"after="<<b.reason<<" land="<<b.request_land<<" vx="<<b.velocity_target[0]<<"\n";
 return b.request_land?0:2;
}
