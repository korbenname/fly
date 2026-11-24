// 仿真适配器。位置、速度和起飞斜坡由指定定制 PX4 源码执行。
// 不链接Commander、EKF、通信、执行器或任何可操作真实飞机的入口。
#include "PositionControl.hpp"
#include "Takeoff.hpp"
#include "px4_model_parameters.hpp"

#include <algorithm>
#include <cmath>

namespace {
struct Model {
  PositionControl control;
  TakeoffHandling takeoff;
  hrt_abstime time_us{1000000};

  Model() {
    using namespace px4_model_parameters;
    control.setPositionGains({MPC_XY_P, MPC_XY_P, MPC_Z_P});
    control.setVelocityGains(
        {MPC_XY_VEL_P_ACC, MPC_XY_VEL_P_ACC, MPC_Z_VEL_P_ACC},
        {MPC_XY_VEL_I_ACC, MPC_XY_VEL_I_ACC, MPC_Z_VEL_I_ACC},
        {MPC_XY_VEL_D_ACC, MPC_XY_VEL_D_ACC, MPC_Z_VEL_D_ACC});
    control.setHoverThrust(MPC_THR_HOVER);
    control.setHorizontalThrustMargin(MPC_THR_XY_MARG);
    control.decoupleHorizontalAndVecticalAcceleration(MPC_ACC_DECOUPLE != 0.f);
    takeoff.setSpoolupTime(COM_SPOOLUP_TIME);
    takeoff.setTakeoffRampTime(MPC_TKO_RAMP_T);
    takeoff.generateInitialRampValue(MPC_Z_VEL_P_ACC);
  }
};
}  // namespace

extern "C" {
void* fc01_test_px4_create() {
  return new Model;
}

void fc01_test_px4_destroy(void* handle) {
  delete static_cast<Model*>(handle);
}

// 仅复现 MulticopterPositionControl::Run 内与 Offboard 起飞和求解器输入相关的分支。
// 简化地面检测器由模型注入。地面接触等同 landed，倾角限值直接设定。
// 启动的 setpoint 新鲜度、通讯和状态确认由 ROS 测试外层校验。
int fc01_test_px4_step(void* handle, float dt, const float* position, const float* velocity,
                       const float* acceleration, float yaw, const float* setpoint,
                       bool armed, bool offboard, bool airborne, float* thrust) {
  if (!handle || !std::isfinite(dt) || dt <= 0.f || dt > .1f)
    return -1;
  using namespace px4_model_parameters;
  auto& model = *static_cast<Model*>(handle);
  model.time_us += static_cast<hrt_abstime>(std::lround(dt * 1e6f));
  if (!offboard || !armed) {
    model.takeoff.updateTakeoffState(armed, !airborne, false, 10.f, true, model.time_us);
    model.control.resetIntegral();
    std::fill_n(thrust, 3, 0.f);
    return 0;
  }

  auto target = PositionControl::empty_trajectory_setpoint;
  std::copy_n(setpoint, 3, target.position);
  std::copy_n(setpoint + 3, 3, target.velocity);
  std::copy_n(setpoint + 6, 3, target.acceleration);
  target.yaw = setpoint[9];
  target.yawspeed = setpoint[10];
  const bool want_takeoff =
      (std::isfinite(target.position[2]) && target.position[2] < position[2]) ||
      (std::isfinite(target.velocity[2]) && target.velocity[2] < 0.f) ||
      (std::isfinite(target.acceleration[2]) && target.acceleration[2] < 0.f);
  model.takeoff.updateTakeoffState(armed, !airborne, want_takeoff, MPC_Z_VEL_MAX_UP,
                                   COM_THROW_EN != 0.f, model.time_us);
  const auto stage = model.takeoff.getTakeoffState();
  const bool flying = stage >= TakeoffState::flight;
  if (stage == TakeoffState::rampup && std::isfinite(target.velocity[2]))
    target.acceleration[2] = NAN;
  if (stage < TakeoffState::rampup || (flying && !airborne)) {
    target = PositionControl::empty_trajectory_setpoint;
    target.acceleration[0] = target.acceleration[1] = 0.f;
    target.acceleration[2] = 100.f;
    model.control.resetIntegral();
  }
  if (!std::isfinite(target.position[0]) && !std::isfinite(target.velocity[0]))
    model.control.resetIntegralXY();
  constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
  model.control.setTiltLimit(
      (flying ? MPC_TILTMAX_AIR : MPC_TILTMAX_LND) * degrees_to_radians);
  model.control.setThrustLimits(flying ? MPC_THR_MIN : 0.f, MPC_THR_MAX);
  model.control.setVelocityLimits(MPC_XY_VEL_MAX,
      std::min(MPC_Z_VEL_MAX_UP, model.takeoff.updateRamp(dt, MPC_Z_VEL_MAX_UP)),
      MPC_Z_VEL_MAX_DN);
  const PositionControlStates states{matrix::Vector3f(position), matrix::Vector3f(velocity),
                                     matrix::Vector3f(acceleration), yaw};
  model.control.setState(states);
  model.control.setInputSetpoint(target);
  if (!model.control.update(dt))
    return -2;
  vehicle_local_position_setpoint_s output{};
  model.control.getLocalPositionSetpoint(output);
  std::copy_n(output.thrust, 3, thrust);
  return 0;
}
}
