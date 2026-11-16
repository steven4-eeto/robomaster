/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-16 02:35:51
 * @FilePath: \robomaster\Gimbal\Instance\component\ins_pid.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "ins_pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace {

hello_world::pid::Pid::Config MakeYawPidConfig() {
  hello_world::pid::Pid::Config cfg;
  /* 位置环刚度。DM4310 直驱、TMax=7.5N·m：
   *   kp=6  → 1° 误差出 0.105N·m，稳态误差 ≈ 摩擦力矩/6（rad）
   *   kp=0.6 时同样的摩擦力矩需要 10 倍误差，云台"发软"、跟随时滞后大，
   *          会让底盘 Follow 环多出一大块相位滞后 → 来回超调。
   * 整定阶梯：6 → 8 → 10，每档确认无啸叫/无高频抖振再加。
   * kd 提供阻尼（本环只有 P+I 时基本无阻尼），用输出低通压住微分噪声。 */
  cfg.kp       = 7.0f;//3
  cfg.ki       = 0.1f;//0.1
  cfg.kd       = 0.4f;//0.2
  cfg.max_out  = 4.0f;//4    // 留 1.5N·m 余量给 TMax=7.5
  cfg.max_iout = 0.8f;//0.8
  cfg.lpf_fc_hz = 30.0f; // 输出一阶低通，~100Hz 对 3Hz 位置环几乎无相位损失
  return cfg;
}

hello_world::pid::Pid::Config MakePitchPidConfig() {
  hello_world::pid::Pid::Config cfg;
  /* pitch 有重力负载，kp 不宜过大；先按 2.5 起步，观察是否点头/抖振。 */
  cfg.kp       = 20.0f;
  cfg.ki       = 0.0f;
  cfg.kd       = 1.0f;
  cfg.max_out  = 6.0f;
  cfg.max_iout = 0.5f;
  cfg.lpf_fc_hz = 10.0f;
  return cfg;
}

}

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorYaw() {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */
  static hello_world::pid::Pid pid(MakeYawPidConfig());
  return &pid;
  //return nullptr;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  /* TODO【新人任务】:
   * 返回 pitch 关节位置环 PID 单例，做法同 GetPidMotorYaw()。
   */
  static hello_world::pid::Pid pid(MakePitchPidConfig());
  return &pid;
  //return nullptr;
}

/* Private function definitions ----------------------------------------------*/
