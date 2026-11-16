/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:36:50
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-16 02:31:20
 * @FilePath: \robomaster\Gimbal\Task\component\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :main_task.cpp
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
#include "main_task.hpp"
#include "comm_task.hpp"
#include "imu.hpp"
#include "robot.hpp"
#include "DT7.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include <chrono>
#include <cmath>
#include "ins_pid.hpp"
#include <sys/time.h>
#include "ins_motor.hpp"
#include "spi.h"
#include "motor_DaMiao.hpp"
#include "module_state.hpp"

robot::GimbalChassisComm comm_(robot::GimbalChassisComm::CodePart::Gimbal, 0x211, 0x210);
robot::Robot robot_;
hello_world::remote_control::DT7 rc_;
hello_world::motor::DM_J4310* yaw_motor_ = nullptr;
hello_world::motor::DM_J4310* pitch_motor_ = nullptr;
hello_world::pid::Pid* yaw_pid_ = nullptr;
hello_world::pid::Pid* pitch_pid_ = nullptr;
robot::Gimbal* gimbal_ = nullptr;

  hello_world::imu::Imu imu_;
hello_world::imu::BMI088HWConfig hw_config;
hello_world::imu::BMI088Config config;
float rot_mat[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
    };

namespace{
  bool first_working_ = false;
  bool last_working_ = false;
  /* 底盘 yaw_ref（deg，世界系）与云台 IMU yaw 的零偏（rad）。
   * 每次进入工作态时重捕获：offset = 底盘目标 - 当前 IMU yaw。
   * 此后有效目标 = 底盘目标 - offset，云台在入口处原地保持，
   * 只跟踪底盘目标的增量，消除上电/急停解除后跳到固定位置的问题。 */
  float yaw_ref_offset_rad_ = 0.0f;
  //float pitch_ref_offset_rad_ = 0.0f;

  /* IMU yaw 正方向与电机角/力矩正方向的一致性系数。
   * 实测（云台无力矩手拨验证）：imu.yaw() 与 angle() 反向，故取 -1.0f。
   * yaw 环反馈 = kImuYawSign * imu.yaw()；取反后环路才是负反馈。 */
  constexpr float kImuYawSign = -1.0f;

  /* imu.yaw()/angle() 输出弧度，而 gimbal_data 的 *_deg 字段及底盘
   * yaw_ref_deg 全链路按度处理，上报前必须换算。 */
  constexpr float kRadToDeg = 57.29577951308232f;

  /* ===== 云台相对底盘的机械角标定项（供底盘推算自身朝向用）=====
   *   rel_deg = kYawMotorSign * (motor_angle_deg - kGimbalCenterDeg)
   * kYawMotorSign    ：把云台往 IMU yaw 增大的方向拨，若 motor angle 也增大取 +1，
   *                    反向减小取 -1（符号错了底盘 Follow 会朝反方向跑/发散）。
   *                    实测 imu.yaw() 与 angle() 反向，与 kImuYawSign 同取 -1.0f：
   *                    rel 必须换算到 IMU（世界）域，使 ψ_g − ψ_c ≡ rel 成立。
   * kGimbalCenterDeg ：云台摆到机械正前方（与车头同向）时读到的 motor angle（deg）。
   *                    它定义了“云台相对底盘 0°”的物理位置，也是 Follow 的对齐目标。
   * 标定方法：Ozone 里手动把云台摆正，读 yaw_motor->angle()*57.29578 填入。 */
  constexpr float kYawMotorSign    = -1.0f;
  constexpr float kGimbalCenterDeg = 0.0f;

  /* 电机角是多圈连续量（angle_range = kNegInfToPosInf），
   * 上报前归一化到 ±180°，否则 int16（±327.67°）会饱和。 */
  inline float wrap180f(float deg) {
    deg = std::fmod(deg, 360.0f);
    if (deg > 180.0f)  deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
  }
}

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */
      robot_.registerDT7(&rc_);

    yaw_motor_ = GetMotorYaw();
    pitch_motor_ = GetMotorPitch();

    yaw_pid_ = GetPidMotorYaw();
    pitch_pid_ = GetPidMotorPitch();

    gimbal_ = new robot::Gimbal(yaw_motor_ , pitch_motor_);
    gimbal_->registerpid(*yaw_pid_ , *pitch_pid_);

    hw_config.hspi = &hspi1;
    
    hw_config.acc_cs_port  = GPIOA;
    hw_config.acc_cs_pin   = GPIO_PIN_4;
    
    hw_config.gyro_cs_port = GPIOB;
    hw_config.gyro_cs_pin  = GPIO_PIN_0;

      imu_.init(hw_config,
      rot_mat, config,
      1000.0f, 0.5f, 0.0f);

      robot_.registerGimbal(gimbal_);
      robot_.registerImu(&imu_);
      robot_.registerYawMotor(yaw_motor_);
      robot_.registerPitchMotor(pitch_motor_);
      robot_.registerGimbalChassisComm(&comm_);

      robot_.getGimbal()->init();

      CommTaskInit();
}

void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
const bool comm_online = !robot_.getGimbalChassisComm()->isOffline();
const bool online_ = robot_.getGimbalChassisComm()->chassis_data().online;

robot_.getImu()->update();

  //robot_.getGimbalChassisComm()->gimbal_data().yaw_deg = robot_.getImu()->yaw();
  //robot_.getGimbalChassisComm()->gimbal_data().pitch_deg = robot_.getImu()->pitch();
  /* yaw 上报世界系角（IMU），与底盘 yaw_ref_deg 同域，且在非工作态
   * 被 freezeRefToGimbalFeedback 冻结成“当前朝向”；
   * pitch 仍上报编码器角，因为 pitch 环工作在车体系。 */
  robot_.getGimbalChassisComm()->gimbal_data().yaw_deg = robot_.getImu()->yaw() * kRadToDeg;
  robot_.getGimbalChassisComm()->gimbal_data().pitch_deg =
      robot_.getPitchMotor()->angle() * kRadToDeg;

  /* 上报“云台相对底盘”的机械转角（deg）。底盘侧用它替代自身 IMU：
   *   yaw_chassis ≈ yaw_deg - gimbal_rel_deg
   * 两者同源（同一个 IMU + 同一台电机编码器），不存在 IMU 安装零位差，
   * 也不会有两颗陀螺零偏不同导致的缓慢漂移。 */
  robot_.getGimbalChassisComm()->gimbal_data().gimbal_rel_deg =
      wrap180f(kYawMotorSign *
               (robot_.getYawMotor()->angle() * kRadToDeg - kGimbalCenterDeg));

  /* 上报 yaw 电机实际输出力矩（N·m，含 motor_info.dir 方向）。
   * 底盘侧用它做「反力矩前馈」：云台对底盘的反力矩 = -T_g，四轮差动产生
   * +T_g 抵消，把云台世界保持环与底盘跟随环物理解耦（路径A）。
   * 用实际力矩（反馈值）而不是指令值：这样前馈自动包含 PI 的修正量与
   * 堵转/饱和效应，无需知道云台环内部参数。 */
  robot_.getGimbalChassisComm()->gimbal_data().gimbal_torque_nm =
      robot_.getYawMotor()->torq();

if (!comm_online || !online_){

  yaw_motor_->setMitInput(yaw_motor_->angle(), 0.0f, 0.0f, 0.0f, 0);
  pitch_motor_->setMitInput(pitch_motor_->angle(), 0.0f, 0.0f, 0.0f, 0);

  last_working_ = false;
  return;
}

  if (robot_.getGimbalChassisComm()->getlifestate() == robot::LifeState::kWorking){

    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

    if (!last_working_){
      first_working_ = true;
      robot_.getGimbal()->init();

      /* 进入工作态瞬间重记零偏：有效目标 = 当前 IMU 朝向，
       * 云台原地保持，不再跳向上电时的固定位置。 */
      yaw_ref_offset_rad_ =
          robot_.getGimbalChassisComm()->chassis_data().yaw_ref_deg * kDegToRad -
          kImuYawSign * robot_.getImu()->yaw();
      //pitch_ref_offset_rad_ =
        //  robot_.getGimbalChassisComm()->chassis_data().pitch_ref_deg * kDegToRad -
          //robot_.getGimbal()->pitchAngle();
    }

  float yaw_ref = robot_.getGimbalChassisComm()->chassis_data().yaw_ref_deg * kDegToRad -
                  yaw_ref_offset_rad_;
  float pitch_ref = robot_.getGimbalChassisComm()->chassis_data().pitch_ref_deg * kDegToRad;//-
                    //pitch_ref_offset_rad_;
  /* yaw 环用 IMU 世界角作反馈：底盘自转时反馈同步变化，
   * 位置环自动反向补偿，云台相对地面保持不动。 */
  robot_.getGimbal()->setYawFeedback(kImuYawSign * robot_.getImu()->yaw());
  robot_.getGimbal()->setTargetAngle(yaw_ref, pitch_ref);
  robot_.getGimbal()->update(0.001);

  yaw_motor_->setMitInput(yaw_ref, 0.0f, 0.0f, 0.0f, robot_.getGimbal()->getyawout());
  //yaw_motor_->setMitInput(yaw_ref, 0.0f, 0.0f, 0.0f, 0);
  pitch_motor_->setMitInput(pitch_ref, 0.0f, 0.0f, 0.0f, robot_.getGimbal()->getpitchout());
  //pitch_motor_->setMitInput(pitch_ref, 0.0f, 0.0f, 0.0f, 0);

last_working_ = true;
first_working_ = false;
  }
  else{

  yaw_motor_->setMitInput(yaw_motor_->angle(), 0.0f, 0.0f, 0.0f, 0);
  pitch_motor_->setMitInput(pitch_motor_->angle(), 0.0f, 0.0f, 0.0f, 0);

    last_working_ = false;
  }
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 htim6 周期中断回调 HAL_TIM_PeriodElapsedCallback：
 *   - 先判中断来自 htim6（避免把别的定时器中断误当成 1ms tick）；
 *   - 再调用 MainTask() 与 CommTask()。
 * 注意：
 *   - main.c 已 HAL_TIM_Base_Start_IT(&htim6)，这里只需补中断里的分发；
 *   - HAL 回调是 C 链接，需用 extern "C" 包裹，否则中断里找不到符号。
 */
volatile bool g_tick_1ms = false;

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {

  if (htim->Instance == TIM6) {
    g_tick_1ms = true;
  }
}
