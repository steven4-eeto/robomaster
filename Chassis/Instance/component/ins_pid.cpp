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
#include <cmath>

/* Private macro -------------------------------------------------------------*/
#define PID_MOTOR_WHEEL_NUM 4

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
namespace hello_world {
namespace pid {

  void Pid::SetConfig(const Config &cfg) { cfg_ = cfg; }
  const Pid::Config& Pid::GetConfig() const { return cfg_; }
  void Pid::SetDt(float dt) { cfg_.dt = dt; }

  void Pid::Reset() {
    integral_ = 0.0f;
    last_err_ = 0.0f;
    last_deriv_ = 0.0f;
    first_ = true;
  }

  float Pid::Calc(float target, float measure) {
    float err = target - measure;

    if (cfg_.dead_zone > 0.0f && std::fabs(err) < cfg_.dead_zone) {
      err = 0.0f;
    }

    const float p = cfg_.kp * err;

    float integral = integral_;
    if (cfg_.integral_sep <= 0.0f || std::fabs(err) <= cfg_.integral_sep) {

      const float ki_eff =
          (integral_ * err < 0.0f) ? cfg_.ki * cfg_.unwind_gain : cfg_.ki;
      integral += ki_eff * err * cfg_.dt;
      if (cfg_.integral_max > 0.0f) {
        if (integral > cfg_.integral_max) integral = cfg_.integral_max;
        if (integral < -cfg_.integral_max) integral = -cfg_.integral_max;
      }
    }

    float deriv = 0.0f;
    if (!first_) {
      deriv = cfg_.kd * (err - last_err_) / cfg_.dt;
      if (cfg_.d_lpf_alpha > 0.0f && cfg_.d_lpf_alpha < 1.0f) {
        deriv = cfg_.d_lpf_alpha * last_deriv_ +
                (1.0f - cfg_.d_lpf_alpha) * deriv;
      }
    }

    float out = p + integral + deriv;

    bool stop_integral = false;
    if (out > cfg_.out_max) {
      out = cfg_.out_max;
      stop_integral = (err > 0.0f);   // 误差与饱和方向同向，停止积分累加
    } else if (out < cfg_.out_min) {
      out = cfg_.out_min;
      stop_integral = (err < 0.0f);
    }
    if (!stop_integral) {
      integral_ = integral;
    }

    last_err_ = err;
    last_deriv_ = deriv;
    first_ = false;

    return out;
  }

}
}

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {
hello_world::pid::Pid::Config MakeWheelCfg(float kp, float ki, float kd);
}

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */
  static hello_world::pid::Pid pids[PID_MOTOR_WHEEL_NUM] = {

      hello_world::pid::Pid(MakeWheelCfg(2000.0f, 800.0f, 0.00f)),
      hello_world::pid::Pid(MakeWheelCfg(2000.0f, 800.0f, 0.00f)),
      hello_world::pid::Pid(MakeWheelCfg(2000.0f, 800.0f, 0.00f)),
      hello_world::pid::Pid(MakeWheelCfg(2000.0f, 800.0f, 0.00f)),
  };

  if (idx < 0 || idx >= PID_MOTOR_WHEEL_NUM) {
    return nullptr;
  }

  return &pids[idx];

  //(void)idx;

  //(void)idx;
  //return nullptr;
}

/* Private function definitions ----------------------------------------------*/
namespace {

hello_world::pid::Pid::Config MakeWheelCfg(float kp, float ki, float kd) {
  hello_world::pid::Pid::Config cfg;

  cfg.kp = kp;
  cfg.ki = ki;
  cfg.kd = kd;
  cfg.dt = kPidDt;
  cfg.out_max = kWheelOutMax;
  cfg.out_min = -kWheelOutMax;
  cfg.integral_max = kWheelIntegralMax;
  cfg.dead_zone = 0.0f;
  cfg.integral_sep = 60.0f;
  cfg.d_lpf_alpha = 0.7f;

  cfg.unwind_gain = 4.0f;

  return cfg;
}
}

extern hello_world::pid::Pid *pid1;
extern hello_world::pid::Pid *pid2;
extern hello_world::pid::Pid *pid3;
extern hello_world::pid::Pid *pid4;
