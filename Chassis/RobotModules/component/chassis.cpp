/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:36:50
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-07 17:04:17
 * @FilePath: \robomaster\Chassis\RobotModules\component\chassis.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :chassis.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"
#include "robot.hpp"

#include <algorithm>
#include <cmath>

extern hello_world::imu::Imu* imu_; 
extern robot::GimbalChassisComm* comm_;

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {
constexpr float kPi = 3.14159265358979323846f;
}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】实现 chassis.hpp 中声明的底盘运动学。
 *
 * 你要做：
 *   1. 逆解：输入 (vx, vy, w)，输出每个轮的轮速 / 电流指令（麦轮逆运动学矩阵）。
 *   2. 对解算结果做限幅，避免超速。
 *
 * 约束 / 提示：
 *   - 解算结果会经电机组件的 encode() 下发，注意输入量的类型（电流 / 转矩）。
 * ==========================================================================*/
namespace robot {

void Chassis::solveLimited(float max_wheel_speed) {
  solve();

  float max_abs = 0.0f;
  for (uint8_t i = 0; i < kWheelNum; ++i) {
    max_abs = std::max(max_abs, std::fabs(wheelSpeed_[i]));
  }

  if (max_abs > max_wheel_speed && max_abs > 1e-6f) {
    const float scale = max_wheel_speed / max_abs;
    for (uint8_t i = 0; i < kWheelNum; ++i) {
      wheelSpeed_[i] *= scale;
    }
  }
}

float Chassis::wheelSpeedToCurrent(float speed_rad_s, float current_per_speed,
                          float max_current) {
  return std::clamp(speed_rad_s * current_per_speed, -max_current, max_current);
}

  void Chassis::setVelocityfs(float vx, float vy, float w){

    constexpr float kDeg2Rad = 0.0174532925f;
    /* 云台相对底盘的机械角（deg -> rad）。这就是原来 wrapPi(yaw_g - yaw_c) 的等价量：
     * 云台世界角 - 底盘世界角 ≡ 云台相对底盘的机械转角。
     * 直接用编码器量，不再读底盘 IMU —— 双板 IMU 坐标系/零位差自然消失。 */
    const float d = wrapPi(comm_->gimbal_data().gimbal_rel_deg * kDeg2Rad);

    const float vx_g = vx;
    const float vy_g = vy;

const float c = cosf(d);
const float s = sinf(d);
    velocity_.vx = c * vx_g - s * vy_g;
    velocity_.vy = s * vx_g + c * vy_g;
    velocity_.w = w;
  }

extern Chassis chassis_;

}
