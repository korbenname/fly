#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main prior_horizontal_test_main
#include "test_horizontal_navigation.cpp"
#undef main
#pragma GCC diagnostic pop
int main(){
 auto original=input();velocity(original,{.2,.1,0});
 h::Controller bad(config());need(bad.step(original,10,1.5,0,.05).active,"starts");
 auto arrived=original;advance(arrived,10.05);
 arrived.velocity.attitude_sample_stamp_ns=10060000000ULL;
 arrived.velocity.matched_attitude_valid=false;
 auto result=bad.step(arrived,10.05,1.5,0,.05);
 need(result.request_land&&!result.active,"future sample absent from native triggers safety");
 std::cout<<"LATEST_BRIDGE_ATTITUDE_AHEAD_OF_NATIVE descends reason="<<result.reason<<"\n";
 h::Controller acked(config());need(acked.step(original,10,1.5,0,.05).active,"starts ack");
 auto good=original;advance(good,10.05);
 good.velocity.attitude_sample_stamp_ns=good.velocity.matched_attitude_stamp_ns=10000000000ULL;
 good.velocity.matched_attitude_received_at=10.;
 result=acked.step(good,10.05,1.5,0,.05);
 need(result.active&&!result.request_land,"known fresh native received sample stays active");
 std::cout<<"EXACT_NATIVE_RECEIVED_FRESH_SAMPLE remains_active\n";
 good.ros_now_ns=10400000000ULL;
 good.actual.local_stamp_ns=good.actual.attitude_stamp_ns=good.ros_now_ns;
 good.actual.local_received_at=good.actual.attitude_received_at=10.4;
 good.velocity.sequence++;good.velocity.stamp_ns=good.ros_now_ns;good.velocity.received_at=10.4;
 result=acked.step(good,10.4,1.5,0,.05);
 need(result.request_land&&!result.active,"old ack sample still expires");
 std::cout<<"ACKNOWLEDGED_SAMPLE_STILL_EXPIRES reason="<<result.reason<<"\n";
}
