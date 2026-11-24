# target pinned interface audit (read-only)

Reference: korbenname/fly @ c4b32b92455a9c25fb1bce1262e8426f249f8521.
Fetched 54 target blobs to `/tmp/fc01-target-pinned/target` via raw.githubusercontent.com; all 54 match Git blob SHA1 from recursive Git tree API (`/tmp/fc01-fly-tree.json`). No AGENTS.md in tree. No hardware connection or live target ROS graph inspected.

## Confirmed message declarations

After comments/whitespace normalization, target VisionState, NavigationCommand and VisionHealth match local declarations exactly. target VelocityCommand additionally declares `float64 wz`; it still calls itself schema_version=1. This is a wire-layout conflict under the same fully-qualified ROS type; do not install two copies of jikl_fc01_system or assume a matching schema_version solves it.

## Endpoint inventory before modification

| Endpoint | Type | Publisher -> subscriber | Meaning/time/frame | QoS |
|---|---|---|---|---|
| /fc01/vision/state (target) | jikl_fc01_system/msg/VisionState | fc01_vision_node or vision_image_node -> mission when use_vision_state=true | Camera-centre-relative target position and relative filtered derivative in body_frd, not self state. stamp from camera read completion (system time) or incoming image header; sequence from pipeline frame_seq. POSITION/VELOCITY/covariance bits set when detected+position_valid. No ATTITUDE/QUALITY. | Both sides best-effort keep-last5 volatile |
| /fc01/vision/state (local) | same VisionState | external self-state adapter -> native visual_inbox_ when enabled | Actual self position/velocity in fc01_vision_ned, FRD->NED wxyz attitude, quality, valid fields. Missing attitude/quality means target payload is unusable. | Native default best-effort depth5 volatile (configurable depth1..20) |
| /fc01/vision/target_relative | geometry_msgs/PointStamped | vision_image_node -> mission | Alternative camera-relative body_frd target point m; invalid point NaN; stamp original image stamp. Direct USB node does not publish it. | best-effort depth5 volatile |
| /visioncontrol | target extended jikl_fc01_system/msg/VelocityCommand | mission -> no local production subscriber | Desired body FRD vx/vy/vz m/s and wz rad/s. FRD yaw positive clockwise viewed above. stamp ROS generation time; sequence starts0. reset_counter from mission resets. | target publisher reliable keep-last10 volatile; no local native subscriber |
| /fc01/vision/velocity_command | local legacy VelocityCommand | external diagnostic caller -> velocity_command_diagnostic.py | Desired body FRD vx/vy/vz; no yaw field; zero control authority | Existing diagnostic profile inspect separately; native has no subscriber |
| /navigation_command | NavigationCommand | mission -> no matching native default subscription | Desired world absolute NED point/yaw (converted from ENU), pre-lock waypoint or valid=false withdrawal. stamp ROS generation time; sequence starts0; mission reset domain. | reliable keep-last10 volatile |
| /fc01/navigation/command | NavigationCommand | external command caller -> native visual_inbox_.command when enabled | Local legacy navigation command, reset must match self VisionState reset. valid=false not a velocity handover implementation | default reliable depth5 volatile (configurable depth1..20) |
| /sim/platform_pose | geometry_msgs/PoseStamped | sim/bridge -> mission | Actual platform world ENU position, body FLU -> world quaternion in ROS xyzw | mission reliable keep-last10 volatile |
| /sim/platform_velocity | geometry_msgs/Vector3Stamped | sim/bridge -> mission | Actual platform ENU velocity m/s | mission reliable keep-last10 volatile |
| /sim/platform_angular_velocity | geometry_msgs/Vector3Stamped | sim/bridge -> mission | Actual body FLU angular velocity rad/s | mission reliable keep-last10 volatile |
| /sim/target_contact_force | std_msgs/Float32 | sim -> mission | Contact magnitude N; no hardware source exists | reliable keep-last10 volatile |
| /sim/impact | std_msgs/Bool | sim -> mission | Contact-ended episode signal; no hardware source exists | reliable keep-last10 volatile |
| /sim/reset_event | std_msgs/Empty | sim -> mission | Reset sim mission/session; no hardware source exists | reliable keep-last10 volatile |
| /mission/state | std_msgs/String | mission -> observers | Human/JSON mission state | reliable keep-last10 volatile |
| /fc01/vision/debug_image | sensor_msgs/Image | vision -> viewers | annotated image with source stamp | direct node reliable keep-last1 volatile, image node reliable keep-last5 volatile |

## Concrete source locations

- `fc01_vision/src/vision_state_adapter.cpp::fillVisionState`: fills target-relative position and velocity into the self-state-shaped message, identity attitude with invalid attitude bit, no quality.
- `fc01_vision/include/fc01_vision/vision_state_adapter.hpp`: explicitly documents camera centre origin and unestimated attitude/quality.
- `fc01_vision/src/camera_source.cpp::captureLoop`: stamp assigned by systemNow after cap.read/optional flip, not proven exposure time.
- `fc01_vision/src/vision_node.cpp::{loadParameters,publishOnce}`: same /fc01/vision/state default; timer republishes cached frame with original stamp/sequence until stale.
- `fc01_mission/fc01_mission/mission_node.py::__init__`: optional imports swallow Exception; pub profile integer10, target sensor profile depth5.
- `MissionNode::_on_state`: only POSITION bit+finite check; resets receive freshness even for duplicate frame; ignores source/reset/frame/tracking/calibration/attitude/quality. Step2 must address true frame freshness.
- `MissionNode::_message_age`: clamps all ages to [0,.15], does not reject old/future source messages; step2 defect.
- `MissionNode::_on_velocity/_on_omega`: no independent timestamp freshness; step2 defect.
- `MissionNode::_publish_command`: computes world ENU -> FLU -> FRD using full rotation; publishes `wz=-yaw_rate_cmd`; no local production implementation yet.
- `MissionNode::_publish_navigation`: ENU -> NED coordinate/yaw conversion; pre-lock publishes takeoff waypoint itself; after lock valid=false. Native handover not implemented; step3/4 scope.
- `fc01_mission/config/mission.yaml`: takeoff_altitude_m=2.78, control_enabled=true, engage_mode=push.
- `fc01_bringup/launch/robot.launch.py`: default true/push; forces use_vision_state=true; no topic arguments/remaps passed; launches USB node.
- `fc01_mission/launch/mission.launch.py`: default true/push; no endpoint args.
- `fc01_vision/CMakeLists.txt`: both USB and image nodes depend on jikl_fc01_system; top comment image node no dependency is stale.
- `target/build.sh`: colcon scans target tree and selects its duplicate jikl_fc01_system; must use the unified workspace entry instead.

## Step1 minimal changes recommended

1. Keep existing local VelocityCommand type byte-identical for legacy diagnostics. Add distinct VelocityCommandV2 with vx/vy/vz/wz to avoid same-type incompatible layout. Explicit wz semantics, source/reset/sequence/stamp/valid preserved.
2. Add distinct TargetObservation type+topic (`/fc01/vision/target_observation`), adapt both target vision nodes and mission subscription, preserve FRD camera-centre origin rather than silently rename to self-state NED.
3. Import fc01_vision/fc01_mission/fc01_bringup source only; build exactly one local jikl_fc01_system package. Do not copy target/src/jikl_fc01_system.
4. Production robot/mission launch must pass explicit configurable target/navigation/velocity endpoint mappings. New velocity receiving profile keep-last1 volatile, publisher compatible. Keep native flight vision disabled and command receive read-only at this stage.
5. Set imported production launch/config to control_enabled=false, headless. Remaining takeoff/navigation assumptions clearly stage3/4 work; do not claim camera can assist flight after step1.
6. Real Humble tests should check normalized declarations, C++/Python type support same install tree, actual fields (including wz), endpoint remaps, QoS via graph, old diagnostic type support, and no native control authority.

## Evidence limitations

No PX4 firmware DDS graph or on-board runtime.yaml was freshly read here. Target reference README/camera numbers are author measurements, not Orange Pi verification. Source endpoint inventory does not establish flight performance or calibration validity.
