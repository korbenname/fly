#include "fc01_vision/vision_pipeline.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
int main(){
 fc01_vision::DetectorConfig d;d.ranges.push_back({0,10,80,255,50,255});d.shape="circle";
 fc01_vision::EstimatorConfig e;e.target_diameter_m=.2;
 fc01_vision::VisionPipeline p(d,e,{});
 fc01_vision::CameraIntrinsics k;k.fx=k.fy=400;k.cx=320;k.cy=240;k.width=640;k.height=480;
 cv::Mat image(480,640,CV_8UC3,cv::Scalar(0,0,0));cv::circle(image,{320,240},40,{0,0,255},-1);
 auto a=p.process(image,k,1000000000);auto b=p.process(image,k,1000000000);
 std::cout<<"same source stamp="<<a.stamp_ns<<"/"<<b.stamp_ns<<" seq="<<a.frame_seq<<"/"<<b.frame_seq<<" valid="<<a.position_valid<<"/"<<b.position_valid<<"\n";
 auto c=p.process(image,k,900000000);
 std::cout<<"regressed source stamp="<<c.stamp_ns<<" seq="<<c.frame_seq<<" valid="<<c.position_valid<<"\n";
}
