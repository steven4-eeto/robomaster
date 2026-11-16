/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:36:50
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-16 02:29:51
 * @FilePath: \robomaster\Gimbal\RobotModules\component\gimbal.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:36:50
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-01 22:54:27
 * @FilePath: \robomaster\Gimbal\RobotModules\component\gimbal.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :gimbal.cpp
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
#include "gimbal.hpp"

#include <cmath>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace robot {
namespace {

constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

constexpr float kPitchLimitMin = -2.00f;
constexpr float kPitchLimitMax =  2.00f;

constexpr float kPitchHardMin = kPitchLimitMin - 0.10f;
constexpr float kPitchHardMax = kPitchLimitMax + 0.10f;

/* ===== pitch 重力前馈 =====
 * 模型：τ_ff = kPitchGravTq · cos(pitch_fb − kPitchGravZero)
 *  - cos 形状：机械零点（pitch=0）在水平方向时，重力矩在水平处最大、
 *    抬头/低头到 ±90° 时为零；
 *  - kPitchGravTq：重力矩幅值（N·m）= m·g·L（云台绕 pitch 轴的
 *    等效质量×质心距离）。标定法：改之前先让 pitch 稳在水平位置，
 *    看 pitch PID 的稳态输出 ≈ 该值（由 ki 慢慢攒出来的那部分）；
 *  - kPitchGravZero：若编码器零点不在水平（设零方式导致的），
 *    填机械水平时 pitch_fb 的读数（rad），让 cos 峰值对准水平位置；
 *  - 符号：若加了 FF 反而点头更厉害 → kPitchGravTq 取负。 */
constexpr float kPitchGravTq   = 4.0f;   // N·m，推荐起点 1.0（典型 RM 云台 0.5~1.5）
constexpr float kPitchGravZero = 0.0f;   // rad，cos 峰值对应的 pitch 角

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}
}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】实现 gimbal.hpp 中声明的云台位置控制。
 *
 * 你要做：
 *   1. yaw 环：目标角与反馈角在同一角度域做差，再经位置环 PID 输出转矩 / 电流。
 *   2. pitch 环：加软件限位，超出范围时钳位指令，防止顶到机械限位堵转。
 *   3. 角度回绕：yaw 的差值要归一到 [-π, π] 区间，避免 359°→1° 时反向大转。
 *
 * 约束 / 提示：
 *   - 位置环输出是转矩 / 电流，注意与电机 setInput 的输入类型匹配。
 * ==========================================================================*/
namespace robot {

Gimbal::Gimbal(hello_world::motor::DM_J4310* yaw_motor, hello_world::motor::DM_J4310* pitch_motor)
    : yaw_motor_(yaw_motor),
      pitch_motor_(pitch_motor),
      yaw_pid_(*GetPidMotorYaw()),
      pitch_pid_(*GetPidMotorPitch()),
      yaw_ref_(0.0f),
      pitch_ref_(0.0f) {}

void Gimbal::init() {
  yaw_pid_.reset();
  pitch_pid_.reset();

  if (yaw_motor_)   yaw_ref_   = normalizeAngle(yaw_motor_->angle());
  if (pitch_motor_) pitch_ref_ = clampf(pitch_motor_->angle(), kPitchLimitMin, kPitchLimitMax);
}

void Gimbal::setTargetAngle(float yaw_ref, float pitch_ref) {

  yaw_ref_ = normalizeAngle(yaw_ref);

  pitch_ref_ = clampf(pitch_ref, kPitchLimitMin, kPitchLimitMax);
}

void Gimbal::setYawFeedback(float yaw_rad) {
  yaw_fb_ext_ = yaw_rad;
  yaw_fb_ext_valid_ = true;
}

void Gimbal::update(float dt) {

    /* 有外部世界系反馈（IMU yaw）时用它，实现对地稳定：
     * 底盘转动会直接改变反馈值，位置环自动反向补偿。
     * 没有则退回电机编码器反馈（车体系）。 */
    const float yaw_fb = yaw_fb_ext_valid_ ? normalizeAngle(yaw_fb_ext_)
                                           : normalizeAngle(yaw_motor_->angle());

    yaw_out = yaw_pid_.calc(yaw_ref_, yaw_fb);

    float pitch_fb = pitch_motor_->angle();
    pitch_fb = normalizeAngle(pitch_fb);

    if (pitch_fb < kPitchHardMin || pitch_fb > kPitchHardMax) {

      pitch_pid_.reset();
      /* 硬限位区不加重力前馈：此时已顶到机械限位，FF 只会往限位上压 */
      pitch_out = 0;
    } else {
      pitch_out = pitch_pid_.calc(pitch_ref_, pitch_fb)
                + kPitchGravTq;// * cosf(pitch_fb - kPitchGravZero);
    }
}

float Gimbal::yawAngle() const {
  return yaw_motor_ ? yaw_motor_->angle() : 0.0f;
}

float Gimbal::pitchAngle() const {
  return pitch_motor_ ? pitch_motor_->angle() : 0.0f;
}

float Gimbal::yawTarget() const { return yaw_ref_; }

float Gimbal::pitchTarget() const { return pitch_ref_; }

float Gimbal::normalizeAngle(float angle) {

  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0f) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

}
