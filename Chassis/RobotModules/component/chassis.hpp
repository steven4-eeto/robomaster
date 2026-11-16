/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-07 17:29:13
 * @FilePath: \robomaster\Chassis\RobotModules\component\chassis.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <gimbal_chassis_comm.hpp>
#include <imu.hpp>
#include <cmath>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】声明底盘运动模块类 Chassis（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Chassis 类，负责把底盘速度指令 (vx, vy, w) 逆解为每个轮的控制量：
 *      麦克纳姆轮运动学逆解（4 个 M3508 轮电机），输出每个轮的轮速。
 *   2. 提供接口让 Robot 下发目标速度、读取解算结果。
 *
 * 约束 / 提示：
 *   - 几何参数（轮半径、轮距、轮心距、轮位坐标）见考核说明文档。
 *   - 逆解公式需自己推导（注意麦轮斜向布置角），网上「舵向轮 / 麦克纳姆轮逆运动学」资料可参考。
 * ==========================================================================*/
namespace robot {

  class Robot; 

class Chassis {
 public:

  enum WheelIndex : uint8_t {
    kWheelLF = 1,
    kWheelRF = 0,
    kWheelLB = 2,
    kWheelRB = 3,
    kWheelNum = 4,
  };

  struct Param {
    float wheelRadius;
    float halfWheelBase;
    float halfTrack;

    /* 几何参数：r=轮半径, l=半轴距(前后轮距一半), w=半轮距(左右轮距一半)
     * 实测：r=0.07786m, l=0.15338m, w=0.15338m。
     * solve() 中 k = l + w = 0.30676m，旋转轮速目标 = k·w_cmd/r ≈ 3.94·w_cmd。 */
    Param(float r = 0.07786f, float l = 0.15338f, float w = 0.15338f)
        : wheelRadius(r), halfWheelBase(l), halfTrack(w) {}
  };

  struct Velocity {
    float vx = 0.0f;
    float vy = 0.0f;
    float w = 0.0f;
  };

  Chassis() = default;
  explicit Chassis(const Param& param, hello_world::imu::Imu* imu_,
  GimbalChassisComm* comm_) : param_(param), imu_(imu_),comm_(comm_) {}
  ~Chassis() = default;

  void setParam(const Param& param) { param_ = param; }
  const Param& param() const { return param_; }

  void setVelocity(float vx, float vy, float w) {
    velocity_.vx = vx;
    velocity_.vy = vy;
    velocity_.w = w;
  }

inline float wrapPi(float angle) {
    return std::remainder(angle, 2.0f * static_cast<float>(M_PI));
}
  
  void setVelocityfs(float vx, float vy, float w);

  void setVelocity(const Velocity& vel) { velocity_ = vel; }
  const Velocity& velocity() const { return velocity_; }

  void solve() {
    const float k = param_.halfWheelBase + param_.halfTrack;
    const float r = param_.wheelRadius;

    const float vx = velocity_.vx;
    const float vy = velocity_.vy;
    const float w = velocity_.w;

    wheelSpeed_[kWheelLF] = (vx - vy - k * w) / r;
    wheelSpeed_[kWheelRF] = (vx + vy + k * w) / r;
    wheelSpeed_[kWheelLB] = (vx + vy - k * w) / r;
    wheelSpeed_[kWheelRB] = (vx - vy + k * w) / r;
  }

  void solveLimited(float max_wheel_speed);

  float wheelSpeed(WheelIndex idx) const {
    return wheelSpeed_[static_cast<uint8_t>(idx)];
  }
  const float* wheelSpeeds() const { return wheelSpeed_; }

  float wheelSpeedToRpm(float speed_ms) const {
    return speed_ms / param_.wheelRadius * 60.0f / 6.283185307f;
  }

  float wheelSpeedToCurrent(float speed_rad_s, float current_per_speed,
                          float max_current);

 private:
  Param param_{};
  Velocity velocity_{};
  float wheelSpeed_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};

  hello_world::imu::Imu* imu_;  
  GimbalChassisComm* comm_;
};

}

 #endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
