/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-08 16:45:42
 * @FilePath: \robomaster\Gimbal\Instance\component\ins_pid.hpp
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
#ifndef GIMBAL_INSTANCE_INS_PID_HPP_
#define GIMBAL_INSTANCE_INS_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace {

constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

}

/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace pid {
class Pid {
 public:
  struct Config {
    float kp       = 0.0f;
    float ki       = 0.0f;
    float kd       = 0.0f;
    float max_out  = 1.0e9f;
    float max_iout = 1.0e9f;
    /* 输出一阶低通截止频率（Hz）。<= 0 表示不滤波。
     * 用于抑制编码器量化噪声经 kd 放大后产生的力矩抖振，
     * 同时让力矩指令平滑，减少机械冲击与啸叫。
     * 取值建议 60~150 Hz：太低（<30Hz）会给位置环引入明显相位滞后，
     * 反而更容易振荡。 */
    float lpf_fc_hz = 0.0f;
  };

  Pid() = default;
  explicit Pid(const Config& cfg) : cfg_(cfg) {}

  void setConfig(const Config& cfg) { cfg_ = cfg; }
  const Config& config() const { return cfg_; }

  void reset() {
    err_[0] = err_[1] = 0.0f;
    p_out_ = i_out_ = d_out_ = out_ = 0.0f;
    out_lpf_ = 0.0f;
    first_ = true;
  }

  float calc(float set, float feedback) {

    err_[1] = err_[0];
    err_[0] = normalizeangle(set - feedback);

    p_out_ = cfg_.kp * err_[0];

    /* 首拍 err_[1] 还是 0，直接做差会得到 err/0.001 的巨大微分尖峰，
     * 经 kd 放大后变成一次冲击力矩，因此首拍微分强制为 0。 */
    const float d_err = first_ ? 0.0f : normalizeangle(err_[0] - err_[1]);
    first_ = false;
    d_out_ = cfg_.kd * d_err / 0.001f;

    float i_new = i_out_ + cfg_.ki * err_[0] * 0.001;
    if (i_new >  cfg_.max_iout) i_new =  cfg_.max_iout;
    if (i_new < -cfg_.max_iout) i_new = -cfg_.max_iout;

    const float out_tmp = p_out_ + i_new + d_out_;
    const bool saturated = (out_tmp >=  cfg_.max_out) || (out_tmp <= -cfg_.max_out);
    const bool pushing   = (out_tmp > 0.0f && err_[0] > 0.0f) ||
                           (out_tmp < 0.0f && err_[0] < 0.0f);
    if (!(saturated && pushing)) {
      i_out_ = i_new;
    }

    out_ = p_out_ + i_out_ + d_out_;
    if (out_ >  cfg_.max_out) out_ =  cfg_.max_out;
    if (out_ < -cfg_.max_out) out_ = -cfg_.max_out;

    /* 输出一阶低通：y(k) = y(k-1) + a*(u(k) - y(k-1))
     *   tau   = 1 / (2*pi*fc)
     *   a     = dt / (tau + dt)
     * 放在限幅之后，保证滤波后的输出仍在 max_out 内。 */
    if (cfg_.lpf_fc_hz > 0.0f) {
      const float tau   = 1.0f / (kTwoPi * cfg_.lpf_fc_hz);
      const float alpha = kSampleTime / (tau + kSampleTime);
      out_lpf_ += alpha * (out_ - out_lpf_);
      out_ = out_lpf_;
    } else {
      out_lpf_ = out_;
    }

    return out_;
  }

float normalizeangle(float angle) {

  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0f) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

  float pOut() const { return p_out_; }
  float iOut() const { return i_out_; }
  float dOut() const { return d_out_; }
  float out()  const { return out_;  }

 private:
  /* 采样周期：MainTask 由 TIM6 以 1ms 触发，与 calc() 内的 0.001f 保持一致 */
  static constexpr float kSampleTime = 0.001f;

  Config cfg_{};
  float err_[2] = {0.0f, 0.0f};
  float p_out_ = 0.0f;
  float i_out_ = 0.0f;
  float d_out_ = 0.0f;
  float out_   = 0.0f;
  float out_lpf_ = 0.0f;   ///< 输出低通状态（即实际下发的力矩）
  bool  first_ = true;     ///< 首拍标记，用于屏蔽微分尖峰
};
}  // namespace pid
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取 yaw 关节位置环 PID 单例。 */
hello_world::pid::Pid *GetPidMotorYaw();
/* 获取 pitch 关节位置环 PID 单例。 */
hello_world::pid::Pid *GetPidMotorPitch();

#endif /* GIMBAL_INSTANCE_INS_PID_HPP_ */
