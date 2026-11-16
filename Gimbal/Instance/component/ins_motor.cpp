/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-08 19:30:21
 * @FilePath: \robomaster\Gimbal\Instance\component\ins_motor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

constexpr uint8_t kYawMotorId   = 0x02;
constexpr uint8_t kPitchMotorId = 0x01;

constexpr float kYawReductionRatio   = 1.0f;
constexpr float kPitchReductionRatio = 1.0f;

constexpr bool  kYawReverse       = false;
constexpr bool  kPitchReverse     = true;
constexpr float kYawAngleOffset   = -1.36920f;
constexpr float kPitchAngleOffset =  0.02480f;

constexpr float kPitchLimitMin = -1.2f;
constexpr float kPitchLimitMax =  1.2f;

constexpr float kDM_PMax = 3.141593f;
constexpr float kDM_VMax = 21.0f;
constexpr float kDM_TMax = 7.5f;

}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::DM_J4310 *GetMotorYaw() {
  /* TODO【新人任务】:
   * 返回云台 yaw 关节电机单例（DM-J4310）。
   *
   * 你要做：
   *   1. 静态定义 yaw 电机对象（CreateMotor(kDM_J4310, id, opt) 或直接构造 DM_J4310）。
   *   2. 配置 id / 方向 / 减速比 / 角度偏移，并确保使能（auto_enable / enable()），
   *      达妙电机失能状态下不会反馈数据。
   *   3. 返回其地址。
   *
   * 约束 / 提示：
   *   - 设备类见 inc/motor_DaMiao.hpp（DM_J4310），工厂 MotorType::kDM_J4310。
   *   - yaw 关节注意 360° 连续旋转的角度过零处理（MIT 位置控制内部不做多圈累计，
   *     见 motor_DaMiao.hpp 注释）。
   */
  static hello_world::motor::DM_J4310 yaw_motor;
  static bool inited = false;

  if (!inited) {
    hello_world::motor::OptionalParams opt;
  opt.dir = hello_world::motor::kDirFwd;
opt.angle_offset = -1.36920f;

    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.max_torq_input_lim = kDM_TMax;

    yaw_motor.init(kYawMotorId, opt, true, false);
    inited = true;
  }

  return &yaw_motor;
  //return nullptr;
}

hello_world::motor::DM_J4310 *GetMotorPitch() {
  /* TODO【新人任务】:
   * 返回云台 pitch 关节电机单例（DM-J4310），做法同 GetMotorYaw()。
   * pitch 关节需用反馈角度做限位，防止堵转 / 撞结构。
   */
  static hello_world::motor::DM_J4310 pitch_motor;
  static bool inited = false;

  if (!inited) {
    hello_world::motor::OptionalParams opt;
  opt.dir = hello_world::motor::kDirRev;
opt.angle_offset = 0.02480f;

    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.max_torq_input_lim = kDM_TMax;

    pitch_motor.init(kPitchMotorId, opt, true, false);
    inited = true;
  }

  return &pitch_motor;
  //return nullptr;
}

/* Private function definitions ----------------------------------------------*/
