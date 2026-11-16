/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:33:38
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-03 09:32:53
 * @FilePath: \robomaster\Gimbal\inc\imu.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（新人填写）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-12-01      Jinletian       1. 初版编写完成
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：掏空 Imu 封装类，
 *                                                只保留 BMI088 芯片驱动 + Mahony 姿态
 *******************************************************************************
 * @attention :
 *  本文件已掏空。新人需基于以下两个已提供的组件自行实现 Imu 封装类：
 *    - BMI088：芯片级原始读取（加速度 m/s² / 角速度 rad/s / 温度）
 *    - ahrs::Mahony：姿态解算（输入角速度 + 加速度，输出四元数/姿态角）
 *  需要新人实现：
 *    - 开机零漂计算（静止采样求角速度均值）
 *    - 姿态角输出 roll()/pitch()/yaw()
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "mahony.hpp"
#include <cstdint>

/* ============================================================================
 * 【新人任务】在本文件编写 Imu 封装类（namespace hello_world::imu）。
 *
 * 你要做：
 *   1. 组合已提供的 BMI088（芯片原始读取，加速度 m/s² / 角速度 rad/s，见 BMI088.hpp）
 *      与 Mahony（姿态解算 update(acc, gyro) → getEulerAngle，见 mahony.hpp / ahrs.hpp）。
 *   2. 实现开机零漂：静止采样一段时间的角速度均值，后续每帧扣除该零漂再送入 Mahony。
 *   3. 对外提供 roll() / pitch() / yaw()（单位 rad），以及标定是否完成的查询。
 *
 * 约束 / 提示：
 *   - Mahony 的加速度输入需含重力加速度项，单位与坐标系（x 前 y 左 z 上）见 ahrs_base.hpp。
 *   - 零漂未收敛前，底盘 / 云台状态机应保持在 Dead / Resurrection，不得进入 Working。
 *   - 本骨架只提供了 imu.hpp（无 imu.cpp）；若需 .cpp 实现文件，可自行在 src/ 下新增
 *     imu.cpp（CMake 已递归 glob *.c*，无需改配置）。
 * ==========================================================================*/
namespace hello_world {
namespace imu {

class Imu {
 public:

   explicit Imu(uint32_t calib_samples = 2000)
      : calib_samples_(calib_samples == 0U ? 1U : calib_samples) {}

  void init(const BMI088HWConfig& hw_config,
            const float rot_mat_flatten[9],
            const BMI088Config& config,
            float samp_freq, float kp, float ki) {
    calibrated_ = false;
    calib_count_ = 0U;

    for (uint32_t i = 0; i < kAxisNum; ++i) {
      gyro_sum_[i] = 0.0f;
      gyro_bias_[i] = 0.0f;
      gyro_corr_[i] = 0.0f;
      acc_[i] = 0.0f;
      gyro_[i] = 0.0f;
      euler_[i] = 0.0f;
    }

    roll_ = pitch_ = yaw_ = 0.0f;

    bmi088_.init(hw_config, rot_mat_flatten, config);
    mahony_.init(samp_freq, kp, ki);
  }

  bool update() {
    bmi088_.getData(acc_, gyro_, &temp_);

    if (!calibrated_) {
      // 开机静止采样，累加角速度
      for (uint32_t i = 0; i < kAxisNum; ++i) {
        gyro_sum_[i] += gyro_[i];
      }
      ++calib_count_;

      if (calib_count_ >= calib_samples_) {
        const float inv_count = 1.0f / static_cast<float>(calib_count_);

        for (uint32_t i = 0; i < kAxisNum; ++i) {
          gyro_bias_[i] = gyro_sum_[i] * inv_count;
          gyro_corr_[i] = gyro_[i] - gyro_bias_[i];
        }

        calibrated_ = true;

        // 用标定完成时的当前帧初始化 Mahony
        mahony_.update(acc_, gyro_corr_);
        updateEuler();
      }

      return true;
    }

    // 正常运行时每帧扣除零漂
    for (uint32_t i = 0; i < kAxisNum; ++i) {
      gyro_corr_[i] = gyro_[i] - gyro_bias_[i];
    }

    mahony_.update(acc_, gyro_corr_);
    updateEuler();
    return true;
  }

  /* 姿态角输出，单位 rad */
  float roll() const { return roll_; }
  float pitch() const { return pitch_; }
  float yaw() const { return yaw_; }

  /* 标定状态查询 */
  bool isCalibrated() const { return calibrated_; }
  bool calibrationDone() const { return calibrated_; }

  uint32_t calibrationCount() const { return calib_count_; }
  uint32_t calibrationTarget() const { return calib_samples_; }

  /**
   * @brief 重新开始零漂标定
   * @note 调用后需保证 IMU 静止，直到 isCalibrated() 返回 true
   */
  void resetCalibration() {
    calibrated_ = false;
    calib_count_ = 0U;

    for (uint32_t i = 0; i < kAxisNum; ++i) {
      gyro_sum_[i] = 0.0f;
      gyro_bias_[i] = 0.0f;
      gyro_corr_[i] = 0.0f;
        euler_[i] = 0.0f;
    }

    roll_ = 0.0f;
    pitch_ = 0.0f;
    yaw_ = 0.0f;

    mahony_.init(1000, 0.5, 0);
  }

 private:
  static constexpr uint32_t kAxisNum = 3U;

  void updateEuler() {
float euler[3] = {0.0f, 0.0f, 0.0f}; 

mahony_.getEulerAngle(euler);

roll_  = euler[0];
pitch_ = euler[1];
yaw_   = euler[2];
  }

  BMI088 bmi088_{};
  ahrs::Mahony mahony_{};

  uint32_t calib_samples_;
  uint32_t calib_count_ = 0U;
  bool calibrated_ = false;

  float gyro_sum_[kAxisNum] = {0.0f, 0.0f, 0.0f};
  float gyro_bias_[kAxisNum] = {0.0f, 0.0f, 0.0f};
  float gyro_corr_[kAxisNum] = {0.0f, 0.0f, 0.0f};

  float acc_[kAxisNum] = {0.0f, 0.0f, 0.0f};
  float gyro_[kAxisNum] = {0.0f, 0.0f, 0.0f};
  float euler_[kAxisNum] = {0.0f, 0.0f, 0.0f};

  float roll_ = 0.0f;
  float pitch_ = 0.0f;
  float yaw_ = 0.0f;

  float temp_ = 0.0f;
};

}
}

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
