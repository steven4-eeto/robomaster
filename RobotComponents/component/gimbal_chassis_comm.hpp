/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
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
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "tick.hpp"
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  static constexpr uint8_t kFrameSize = 8;
  static constexpr uint32_t kOfflineThresholdMs = 100;
  static constexpr float kAngleDegPerLsb = 0.01f;

  /* 底盘→云台（C2G）帧数据：字段由新人按考核要求设计。 */
  struct ChassisData {
    /* TODO【新人任务】:
     * 在此定义 C2G 方向需要传输的语义字段，至少覆盖：
     *   - 底盘是否在线 / 就绪
     *   - 底盘当前工作模式
     *   - 底盘期望的云台 yaw / pitch 角度
     * （共享枚举建议定义在 module_state.hpp 并在本文件 include）
     */
    bool online = false;
    bool ready = false;
    ChassisMode mode = ChassisMode::Separate;
    float yaw_ref_deg = 0.0f;
    float pitch_ref_deg = 0.0f;
  };
  /* 云台→底盘（G2C）帧数据：字段由新人按考核要求设计。 */
  struct GimbalData {
    /* TODO【新人任务】:
     * 在此定义 G2C 方向需要传输的语义字段，至少覆盖：
     *   - 云台是否在线 / 就绪
     *   - 云台当前工作模式
     *   - 云台实际 yaw / pitch 角度反馈
     */
    bool online = false;
    bool ready = false;
    GimbalMode mode = GimbalMode::Joint;
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;   
    /* 云台相对底盘的机械转角（deg，已归一化到 ±180，已按世界系正方向取号、
     * 并扣掉机械中位）。底盘用它 + yaw_deg 反推自身世界朝向：
     *   yaw_chassis ≈ yaw_deg - gimbal_rel_deg
     * 该值来自电机编码器，不受双板 IMU 坐标系/零偏差异影响。 */
    float gimbal_rel_deg = 0.0f;
    /* 云台 yaw 电机实际输出力矩（N·m，电机输出轴，已含 motor_info.dir 方向）。
     * 底盘用它做「反力矩前馈」：云台对底盘的反力矩 = -T_g，四轮差动产生 +T_g
     * 抵消，从而把「云台世界保持环」与「底盘跟随环」物理解耦（路径A）。
     * 量化：int8，1/16 N·m（±7.94 N·m 覆盖 DM4310 的 TMAX=7.5）。 */
    float gimbal_torque_nm = 0.0f;
  };

  void csetready(int ready_ ){chassis_data_.ready = ready_;}
  void gsetready(int ready_ ){gimbal_data_.ready = ready_;}

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[8]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。 */
  void decode(const uint8_t rx_data[8]);

  /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
  bool isOffline();

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }
  uint32_t tx_id() { return tx_id_; }
  LifeState getlifestate(){return lifestate_;}
  void setlifestate(LifeState lifestate){lifestate_ = lifestate;}

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
  bool isoffline_;
  LifeState lifestate_ = LifeState::kDead;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
