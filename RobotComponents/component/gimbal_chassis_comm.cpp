/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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
#include "gimbal_chassis_comm.hpp"

#include <cstring>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
namespace {
constexpr uint32_t kOfflineThresholdMs = 100;

constexpr float kChassisVelScale = 1000.0f;
constexpr float kChassisAngScale = 1000.0f;
constexpr float kGimbalAngScale = 100.0f;
/* G2C 帧字节紧张（8 字节已满），为腾出 1 字节传力矩，pitch 改为 int8：
 *   0.25°/lsb → 范围 ±31.75°，覆盖 pitch 的 ±20° 限位，仅用于 freeze，精度够用。 */
constexpr float kPitchInt8Scale = 4.0f;    // counts per deg
/* 力矩：1/16 N·m/lsb → ±7.94 N·m，覆盖 DM4310 TMAX=7.5 */
constexpr float kTorqueInt8Scale = 16.0f;  // counts per N·m

inline int16_t saturateToInt16(float v) {
  if (!std::isfinite(v)) {
    return 0;
  }
  if (v > 32767.0f) {
    v = 32767.0f;
  }
  if (v < -32768.0f) {
    v = -32768.0f;
  }
  return static_cast<int16_t>(std::lround(v));
}

inline int16_t floatToInt16(float v, float scale) {
  return saturateToInt16(v * scale);
}

inline float int16ToFloat(int16_t v, float scale) {
  return static_cast<float>(v) / scale;
}

/* int8 有符号限幅（-128..127），用于 pitch / 力矩这类小动态范围量 */
inline int8_t saturateToInt8(float v) {
  if (!std::isfinite(v)) {
    return 0;
  }
  if (v > 127.0f) {
    v = 127.0f;
  }
  if (v < -128.0f) {
    v = -128.0f;
  }
  return static_cast<int8_t>(std::lround(v));
}

}

extern robot::GimbalChassisComm comm_;

robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id) {
  /* TODO【新人任务】:
   * 保存 code_part / tx_id / rx_id，并把 last_rx_tick_ 初始化为当前 GetTickMs()。
   */
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[8]) {
  /* TODO【新人任务】:
   * 编码己方帧：把本地 ChassisData / GimbalData 按 hpp 里定的字段布局打包进 8 字节。
   *   - 由 code_part_ 决定打包哪个方向（Chassis → ChassisData，Gimbal → GimbalData）。
   *   - 浮点量按约定比例缩放成整数，避免丢位；收发两端比例必须一致。
   *   - 打包完成后由 Task/component/comm_task.cpp 用 tx_id_ 发送。
   */
  //(void)tx_data;

  if (tx_data == nullptr) {
    return;
  }

  memset(tx_data, 0, 8);

  if (code_part_ == CodePart::Chassis) {

    const int16_t yaw =
        floatToInt16(chassis_data_.yaw_ref_deg, kGimbalAngScale);
    const int16_t pitch =
        floatToInt16(chassis_data_.pitch_ref_deg, kGimbalAngScale);

    tx_data[0] = chassis_data_.online ? 1u : 0u;
    tx_data[1] = chassis_data_.ready ? 1u : 0u;
    tx_data[2] = static_cast<uint8_t>(lifestate_);
    tx_data[3] = static_cast<uint8_t>(yaw & 0xFF);
    tx_data[4] = static_cast<uint8_t>((yaw >> 8) & 0xFF); 
    tx_data[5] = static_cast<uint8_t>(pitch & 0xFF);
    tx_data[6] = static_cast<uint8_t>((pitch >> 8) & 0xFF);     
    tx_data[7] = 0;
  } else {

    const int16_t yaw = floatToInt16(gimbal_data_.yaw_deg, kGimbalAngScale);
    const int16_t rel = floatToInt16(gimbal_data_.gimbal_rel_deg, kGimbalAngScale);
    /* pitch 压缩成 int8（0.25°/lsb）腾出 byte3；byte3 传 yaw 电机力矩（1/16 N·m） */
    const int8_t pitch = saturateToInt8(gimbal_data_.pitch_deg * kPitchInt8Scale);
    const int8_t torq = saturateToInt8(gimbal_data_.gimbal_torque_nm * kTorqueInt8Scale);

    tx_data[0] = static_cast<uint8_t>(yaw & 0xFF);
    tx_data[1] = static_cast<uint8_t>((yaw >> 8) & 0xFF); 
    tx_data[2] = static_cast<uint8_t>(pitch);
    /* byte3: 云台 yaw 电机输出力矩，int8，1/16 N·m（底盘用于反力矩前馈） */
    tx_data[3] = static_cast<uint8_t>(torq);
    tx_data[4] = static_cast<uint8_t>(gimbal_data_.mode);
    /* byte5: bit0 = online, bit1 = ready（原来各占一字节，现合并以腾出空间） */
    tx_data[5] = static_cast<uint8_t>((gimbal_data_.online ? 0x01u : 0x00u) |
                                      (gimbal_data_.ready  ? 0x02u : 0x00u));
    /* byte6-7: 云台相对底盘的机械角，单位 0.01° */
    tx_data[6] = static_cast<uint8_t>(rel & 0xFF);
    tx_data[7] = static_cast<uint8_t>((rel >> 8) & 0xFF);
  }

}

inline int16_t readInt16LE(const uint8_t* p) {
  const uint16_t u =
      static_cast<uint16_t>(p[0]) |
      (static_cast<uint16_t>(p[1]) << 8);
  return static_cast<int16_t>(u);
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  /* TODO【新人任务】:
   * 解码对方帧：把 8 字节按同样布局还原成对方语义，更新本地缓存，并刷新 last_rx_tick_。
   *   - 严格与 encode() 对称；任何一端偏移写错都会让对面读到错乱数据。
   *   - 建议先写自测：同一结构 encode 后再 decode 应能还原原值。
   */
  //(void)rx_data;

  if (rx_data == nullptr) {
    return;
  }

  if (code_part_ == CodePart::Chassis) {

    gimbal_data_.yaw_deg =
        int16ToFloat(readInt16LE(&rx_data[0]), kGimbalAngScale);
    /* byte2: pitch int8 @0.25°；byte3: yaw 电机力矩 int8 @1/16 N·m */
    gimbal_data_.pitch_deg =
        static_cast<float>(static_cast<int8_t>(rx_data[2])) / kPitchInt8Scale;
    gimbal_data_.gimbal_torque_nm =
        static_cast<float>(static_cast<int8_t>(rx_data[3])) / kTorqueInt8Scale;
    gimbal_data_.mode = GimbalMode::Joint;
    gimbal_data_.online = (rx_data[5] & 0x01u) != 0;
    gimbal_data_.ready  = (rx_data[5] & 0x02u) != 0;
    gimbal_data_.gimbal_rel_deg =
        int16ToFloat(readInt16LE(&rx_data[6]), kGimbalAngScale);
  } else {
    chassis_data_.online = (rx_data[0] != 0);

    if (rx_data[2]==0){
      lifestate_ = robot::LifeState::kDead;
    }
    else if (rx_data[2]==1){
      lifestate_ = robot::LifeState::kResurrection;
    }
    else
      lifestate_ = robot::LifeState::kWorking;

    chassis_data_.yaw_ref_deg =
        int16ToFloat(readInt16LE(&rx_data[3]), kGimbalAngScale);
    chassis_data_.pitch_ref_deg =
        int16ToFloat(readInt16LE(&rx_data[5]), kGimbalAngScale);
  }

  last_rx_tick_ = hello_world::tick::GetTickMs();

}

bool robot::GimbalChassisComm::isOffline(){
  /* TODO【新人任务】:
   * 用 GetTickMs() 与 last_rx_tick_ 的差值判断对方是否离线（超过阈值即离线）。
   * 阈值可自定义（如 100ms），供状态机 / 看门狗判断对方是否还活着。
   */
  //return false;
  isoffline_ = (hello_world::tick::GetTickMs() - last_rx_tick_) >
         kOfflineThresholdMs;
  return (hello_world::tick::GetTickMs() - last_rx_tick_) >
         kOfflineThresholdMs;
}

/* Private function definitions ----------------------------------------------*/
