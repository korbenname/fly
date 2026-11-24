#!/usr/bin/env python3
"""实际C++图像节点的标定边界回归；只用合成图，不接触相机或飞控。"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expect-unfixed', action='store_true', help='仅复现旧代码缺陷')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('requires isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from jikl_fc01_system.msg import TargetObservation

    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('camera_calibration_fixture')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    info_pub = node.create_publisher(CameraInfo, '/calibration/info', 1)
    image_pub = node.create_publisher(Image, '/calibration/image', 1)
    out, statuses = [], []
    node.create_subscription(TargetObservation, '/fc01/vision/target_observation', out.append, qos)
    node.create_subscription(String, '/fc01/vision/camera_status',
                             lambda m: statuses.append(json.loads(m.data)), 10)
    pixels = np.zeros((120,160,3),np.uint8)
    yy,xx = np.ogrid[:120,:160]
    pixels[(xx-80)**2+(yy-60)**2<=10**2,2] = 255
    base = CameraInfo()
    base.width, base.height = 160,120
    base.k = [100.,0.,80.,0.,100.,60.,0.,0.,1.]
    base.d = [0.,0.,0.,0.,0.]
    base.distortion_model = 'plumb_bob'
    checks, process, log = [], None, None

    def spin(duration=.05):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.005)

    def wait(predicate, duration=6):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            spin(.02)
            if predicate(): return
        raise AssertionError('condition timeout '+repr(statuses[-2:]))

    def frame():
        image=Image()
        image.width,image.height,image.encoding,image.step=160,120,'bgr8',480
        image.header.stamp=node.get_clock().now().to_msg()
        image.data=pixels.tobytes()
        image_pub.publish(image)
        return image.header.stamp.sec*10**9+image.header.stamp.nanosec

    def detection():
        stamp=frame()
        wait(lambda:out and out[-1].stamp.sec*10**9+out[-1].stamp.nanosec==stamp)
        return out[-1]

    def stop():
        nonlocal process,log
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5)==0
        if log:log.close()
        process=log=None
        spin(.15)

    def start(label, confirm=True):
        nonlocal process,log
        out.clear();statuses.clear()
        params = dict(use_camera_info=True,image_topic='/calibration/image',camera_info_topic='/calibration/info',
            frame_width=160,frame_height=120,camera_fx=100.,camera_fy=100.,camera_cx=80.,camera_cy=60.,
            target_shape='circle',target_diameter_m=.2,calibration_confirmed=True,calibration_id='synthetic-intrinsic',
            image_rectified_confirmed=confirm,extrinsics_confirmed=True,extrinsics_id='synthetic-rotation',
            target_model_confirmed=True,target_model_id='synthetic-circle',capture_latency_confirmed=True,
            capture_latency_bound_s=.01)
        argv=[str(args.binary),'--ros-args']
        for key,value in params.items():
            argv+=['-p',key+':='+(str(value).lower() if isinstance(value,bool) else str(value))]
        log=(args.output/(label+'.log')).open('w')
        process=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT)
        wait(lambda:image_pub.get_subscription_count()==info_pub.get_subscription_count()==1)
        spin(.15)
        info_pub.publish(base)
        spin(.04)

    try:
        start('nonzero-distortion')
        assert detection().tracking
        bad=copy.deepcopy(base);bad.d=[.1,0.,0.,0.,0.]
        info_pub.publish(bad);spin(.04)
        m=detection()
        if args.expect_unfixed:
            assert m.tracking and m.calibration_valid
            result={'reproduced':True,'defect':'nonzero CameraInfo D ignored while calibration_valid remains true',
                    'tracking':m.tracking,'calibration_valid':m.calibration_valid,'hardware':False}
        else:
            assert not m.tracking and not m.calibration_valid
            assert statuses[-1]['reason']=='CAMERA_INFO_DISTORTION_UNSUPPORTED'
            checks.append('nonzero_distortion_revokes')
            info_pub.publish(base);spin(.04)
            assert not detection().calibration_valid
            checks.append('valid_camera_info_cannot_revive_latched_calibration')
            stop()
            cases=[]
            nan_k=copy.deepcopy(base);nan_k.k[0]=float('nan');cases.append(('nan-k',nan_k,'CAMERA_INFO_INTRINSICS_INVALID'))
            nan_d=copy.deepcopy(base);nan_d.d[1]=float('nan');cases.append(('nan-d',nan_d,'CAMERA_INFO_DISTORTION_UNSUPPORTED'))
            changed=copy.deepcopy(base);changed.width=320;cases.append(('resolution',changed,'CAMERA_INFO_CHANGED_RECONFIRM_REQUIRED'))
            changed=copy.deepcopy(base);changed.k[2]=81.;cases.append(('principal-point',changed,'CAMERA_INFO_CHANGED_RECONFIRM_REQUIRED'))
            changed=copy.deepcopy(base);changed.distortion_model='equidistant';cases.append(('unsupported-model',changed,'CAMERA_INFO_DISTORTION_UNSUPPORTED'))
            for label,bad,reason in cases:
                start(label)
                assert detection().tracking
                info_pub.publish(bad);spin(.04)
                assert not detection().calibration_valid
                assert statuses[-1]['reason']==reason,(label,statuses[-1])
                info_pub.publish(base);spin(.04)
                assert not detection().calibration_valid
                checks.append(label+'_reject_and_latch')
                stop()
            start('rectification-unconfirmed',confirm=False)
            assert not detection().calibration_valid
            assert 'CAMERA_RECTIFICATION' in statuses[-1]['missing']
            checks.append('zero_distortion_model_requires_explicit_confirmation')
            result={'passed':True,'checks':checks,'environment':'Humble x86_64 synthetic Image/CameraInfo',
                    'hardware':False,'no_flight_controller':True}
        (args.output/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False))
    finally:
        stop();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
