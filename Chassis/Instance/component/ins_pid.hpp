/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-06 20:55:52
 * @FilePath: \robomaster\Chassis\Instance\component\ins_pid.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_pid.hpp
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
#ifndef CHASSIS_INSTANCE_INS_PID_HPP_
#define CHASSIS_INSTANCE_INS_PID_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace {
constexpr float kPidDt           = 0.001f;
constexpr float kWheelOutMax     = 10000.0f;
constexpr float kWheelIntegralMax = 1500.0f;  // 积分上限=破摩擦水平：任何积分阶跃都不超过破摩擦力矩
} 

/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace pid {

  class Pid {
    public:
  struct Config {
    float kp = 0.0f;              // 比例系数
    float ki = 0.0f;              // 积分系数（已含 dt，直接与误差相乘累加）
    float kd = 0.0f;              // 微分系数（已含 1/dt）
    float dt = kPidDt;            // 采样周期，单位 s
    float out_max = 0.0f;         // 输出上限
    float out_min = 0.0f;         // 输出下限
    float integral_max = 0.0f;    // 积分项限幅，<=0 表示不限幅
    float dead_zone = 0.0f;       // 误差死区，<=0 表示不启用
    float integral_sep = 0.0f;    // 积分分离阈值，<=0 表示不分离
    float d_lpf_alpha = 0.0f;     // 微分一阶低通系数 [0,1)，0 表示不滤波
    /* 非对称积分倍率：误差方向与已有积分相反（需要退积分）时，积分按
     * unwind_gain 倍速退。用途：ki 取大以破静摩擦（慢充），同时用快速
     * 退积分消除破摩擦瞬间/目标阶跃后的超调（快退）。1.0 表示关闭。 */
    float unwind_gain = 1.0f;
  };

  Pid() = default;
  explicit Pid(const Config &cfg) : cfg_(cfg) {}

  void SetConfig(const Config &cfg);
  const Config& GetConfig() const;
  void SetDt(float dt);

  void Reset();

  float Calc(float target, float measure);

 private:
  Config cfg_{};
  float integral_ = 0.0f;      // 积分累积量（已乘 ki*dt）
  float last_err_ = 0.0f;      // 上一拍误差
  float last_deriv_ = 0.0f;    // 上一拍滤波后微分
  bool first_ = true;          // 首拍标记
  };
}  // namespace pid
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取第 idx 号轮电机速度环 PID 单例（idx ∈ [0,4)）。 */
hello_world::pid::Pid *GetPidMotorWheel(int idx);

#endif /* CHASSIS_INSTANCE_INS_PID_HPP_ */
