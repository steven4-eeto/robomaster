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
#include "robot.hpp"
#include "imu.hpp"
#include "spi.h"
#include "DT7.hpp"
#include "chassis.hpp"
#include "gimbal_chassis_comm.hpp"
#include <chrono>
#include "ins_pid.hpp"
#include <sys/time.h>
#include <cmath>

extern "C" int _gettimeofday(struct timeval *tv, void *tz) {
    (void)tz;
    if (tv != nullptr) {
        tv->tv_sec = 0;
        tv->tv_usec = 0;
    }
    return 0;
}

robot::GimbalChassisComm comm_(robot::GimbalChassisComm::CodePart::Chassis, 0x210, 0x211);
robot::Robot robot_;
hello_world::remote_control::DT7 rc_;
hello_world::motor::Motor *m1 = nullptr;
hello_world::motor::Motor *m2 = nullptr;
hello_world::motor::Motor *m3 = nullptr;
hello_world::motor::Motor *m4 = nullptr;
robot::Chassis chassis_(robot::Chassis::Param(), &hello_world::imu::imu_, &comm_);
hello_world::pid::Pid *pid1 = GetPidMotorWheel(0);
hello_world::pid::Pid *pid2 = GetPidMotorWheel(1);
hello_world::pid::Pid *pid3 = GetPidMotorWheel(2);
hello_world::pid::Pid *pid4 = GetPidMotorWheel(3);

namespace hello_world {
namespace imu {

  hello_world::imu::Imu imu_;
BMI088HWConfig hw_config;
BMI088Config config;

}
}

static hello_world::motor::Motor *m[4];
static hello_world::pid::Pid *p[4];

constexpr float kMaxVx = 0.4f;
constexpr float kMaxVy = 0.4f;
constexpr float kMaxW  = 5.0f;
constexpr float kMousePitchSens = 1.0f;
constexpr float kMouseYawSens = 2.0f;
constexpr float kPitchMinDeg = -20.0f;
constexpr float kPitchMaxDeg = 20.0f;

namespace{
  float last_yaw_deg_;
  std::chrono::steady_clock::time_point last_t_;
  float wz;

  float follow_w_prev_ = 0.0f;

  float follow_gz_f_ = 0.0f;
}

namespace{
  bool estop_latched_ = false;
  bool estop_cleared_this_cycle_ = false;
}

namespace{
  robot::ChassisMode last_chassis_mode_ = robot::ChassisMode::Separate;
  robot::ChassisMode now_chassis_mode_ = robot::ChassisMode::Separate;
  bool first_working_ = false;
  bool last_working_ = false;
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
    hello_world::imu::hw_config.hspi = &hspi1;
    
    hello_world::imu::hw_config.acc_cs_port  = GPIOA;
    hello_world::imu::hw_config.acc_cs_pin   = GPIO_PIN_4;
    
    hello_world::imu::hw_config.gyro_cs_port = GPIOB;
    hello_world::imu::hw_config.gyro_cs_pin  = GPIO_PIN_0;

      hello_world::imu::imu_.init(hello_world::imu::hw_config,
      hello_world::imu::rot_mat, hello_world::imu::config,
      1000.0f, 0.5f, 0.0f);

  m1 = GetMotorWheel(0);
  m2 = GetMotorWheel(1);
  m3 = GetMotorWheel(2);
  m4 = GetMotorWheel(3);

  m[0] = m1;
  m[1] = m2;
  m[2] = m3;
  m[3] = m4;

  p[0] = pid1;
  p[1] = pid2;
  p[2] = pid3;
  p[3] = pid4;
    
  robot_.registerWheelMotors(m1, m2, m3, m4);
  robot_.registerImu(&hello_world::imu::imu_);
  robot_.registerDT7(&rc_);
  robot_.registerComm(&comm_);
  robot_.registerChassis(&chassis_);
  robot_.setmotors(m);

  last_yaw_deg_ = 0.0f;
  last_t_ = std::chrono::steady_clock::now();
  wz = 0.0f;

  CommTaskInit();
}

float yawtow(){
  constexpr float kDeg2Rad   = 0.0174532925f;

  constexpr float kKpFollow  = 6.0f;    
  constexpr float kKdFollow  = 0.25f;   
  constexpr float kKiFollow  = 0.0f;    
  constexpr float kDeadDeg   = 0.3f;    

  constexpr float kMaxW      = 3.0f;    

  constexpr float kWiMax     = 0.25f;

  constexpr float kFollowGzSign = 1.0f;
  constexpr float kFollowGzLpf  = 0.2f;

  static float w_i_follow_ = 0.0f;

  constexpr float kUpPerTick   = 0.02f;
  constexpr float kDownPerTick = 0.10f;

  float d = comm_.gimbal_data().gimbal_rel_deg;
  if (d >  180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;

  follow_gz_f_ += kFollowGzLpf * (robot_.imu()->gz() - follow_gz_f_);

  float e = 0.0f;
  if (d >  kDeadDeg) e = (d - kDeadDeg) * kDeg2Rad;
  if (d < -kDeadDeg) e = (d + kDeadDeg) * kDeg2Rad;

  if (e == 0.0f || w_i_follow_ * e < 0.0f) {
    w_i_follow_ = 0.0f;
  } else {
    w_i_follow_ += kKiFollow * e * 0.001f;
    if (w_i_follow_ >  kWiMax) w_i_follow_ =  kWiMax;
    if (w_i_follow_ < -kWiMax) w_i_follow_ = -kWiMax;
  }

  float w_target =
      kKpFollow * e + w_i_follow_ - kKdFollow * kFollowGzSign * follow_gz_f_;
  w_target = std::clamp(w_target, -kMaxW, kMaxW);

  float dw = w_target - follow_w_prev_;
  const bool speeding_up =
      (w_target * follow_w_prev_ >= 0.0f) &&
      (std::fabs(w_target) > std::fabs(follow_w_prev_));
  const float lim = speeding_up ? kUpPerTick : kDownPerTick;
  if (dw >  lim) dw =  lim;
  if (dw < -lim) dw = -lim;
  follow_w_prev_ += dw;
  return follow_w_prev_;
}

static float clamp(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

float wrapTo180(float angle) {
    angle = std::fmod(angle + 180.0f, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle - 180.0f;
}

void resetevery(){
  int i;
  for (i=0; i<4; i++){
    p[i]->Reset();
  }
  last_t_ = std::chrono::steady_clock::now();
  last_yaw_deg_ = comm_.gimbal_data().yaw_deg;
  wz = 0;
  follow_w_prev_ = 0.0f;
  follow_gz_f_ = 0.0f;
}

constexpr float kCoulombRaw = 1200.0f; 
constexpr float kFfBand     = 1.8f;    
constexpr float kFfVisc     = 30.0f;  

constexpr float kGimbalTqGain = 0.0f;
constexpr float kGimbalTqSign = 1.0f;

constexpr float kGimbalTqLpf  = 0.25f;

inline float stictionFF(float tgt) {

  float x = tgt / kFfBand;
  if (x > 1.0f) {
    x = 1.0f;
  } else if (x < -1.0f) {
    x = -1.0f;
  }
  const float s = 1.5f * x - 0.5f * x * x * x;

  return kCoulombRaw * s + kFfVisc * tgt;
}

static void freezeRefToGimbalFeedback() {
  if (robot_.comm() == nullptr) {
    return;
  }
  robot_.comm()->chassis_data().yaw_ref_deg =
      robot_.comm()->gimbal_data().yaw_deg;
  robot_.comm()->chassis_data().pitch_ref_deg =
      clamp(robot_.comm()->gimbal_data().pitch_deg,
            kPitchMinDeg, kPitchMaxDeg);
  robot_.comm()->chassis_data().online = false;
}

void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
const bool rc_online = robot_.getrc()->update(HAL_GetTick());
const bool comm_online = !robot_.comm()->isOffline();

if (rc_.rc_r_switch() == hello_world::remote_control::SwitchState::kSwitchStateDown) {
  estop_latched_ = true;
}

if (estop_latched_ &&
    rc_.rc_r_switch() != hello_world::remote_control::SwitchState::kSwitchStateDown &&
    rc_.estop_clear_btn()) {
  estop_latched_ = false;
  estop_cleared_this_cycle_ = true;
}

if (!estop_latched_ && rc_online && comm_online){
  robot_.comm()->chassis_data().online = true;
}

if (estop_latched_ ||
!rc_online 
|| !comm_online
){
    for (int i = 0; i < 4; ++i) {
    if (m[i]) m[i]->setInput(0.0f);
  }
    freezeRefToGimbalFeedback();
    last_working_ = false;
  return;
}

if (estop_cleared_this_cycle_){
  resetevery();
  estop_cleared_this_cycle_ = false;
}

  robot_.imu()->update();

  robot_.processStateMachine();

  robot_.comm()->setlifestate(robot_.getstate());

  if (robot_.getstate() == robot::LifeState::kWorking){

    if (!last_working_){
      first_working_ = true;
      resetevery();
    }

  switch(rc_.rc_l_switch()){
    case hello_world::remote_control::SwitchState::kSwitchStateUp :{
      robot_.setchassismode(robot::ChassisMode::Separate);
      now_chassis_mode_ = robot::ChassisMode::Separate;
      break;
    }
    case hello_world::remote_control::SwitchState::kSwitchStateMid :{
      robot_.setchassismode(robot::ChassisMode::Follow);
      now_chassis_mode_ = robot::ChassisMode::Follow;
      break;
    }
    case hello_world::remote_control::SwitchState::kSwitchStateDown :{
      robot_.setchassismode(robot::ChassisMode::Spin);
      now_chassis_mode_ = robot::ChassisMode::Spin;
      break;
    }
  }

  if (!first_working_ && last_chassis_mode_ != now_chassis_mode_){
    resetevery();
  }

  switch(robot_.chassisMode()){
    case robot::ChassisMode::Separate :{
      const float vx = (&comm_ != nullptr) ? kMaxVx * rc_.rc_lv() : 0.0f;
      const float vy = (&comm_ != nullptr) ? kMaxVy * rc_.rc_lh() : 0.0f;
      const float wz = (&comm_ != nullptr) ? kMaxW * rc_.rc_rh() : 0.0f;
      robot_.chassis()->setVelocityfs(vx, vy, wz);
      break;
    }
    case robot::ChassisMode::Follow :{
      const float vx = (&comm_ != nullptr) ? kMaxVx * rc_.rc_lv() : 0.0f;
      const float vy = (&comm_ != nullptr) ? kMaxVy * rc_.rc_lh() : 0.0f;
      const float wz = (&comm_ != nullptr) ? yawtow() : 0.0f;
      robot_.chassis()->setVelocityfs(vx, vy, wz);
      break;
    }
    case robot::ChassisMode::Spin :{
      const float vx = (&comm_ != nullptr) ? kMaxVx * rc_.rc_lv() : 0.0f;
      const float vy = (&comm_ != nullptr) ? kMaxVy * rc_.rc_lh() : 0.0f;
      robot_.chassis()->setVelocityfs(vx, vy, 2.0);
      break;
    }
  }

  robot_.chassis()->solve();

  static float gimbal_tq_lpf_ = 0.0f;
  const float tq_in = kGimbalTqSign * comm_.gimbal_data().gimbal_torque_nm;
  gimbal_tq_lpf_ += kGimbalTqLpf * (tq_in - gimbal_tq_lpf_);
  float gimbal_ff_raw = 0.0f;
  if (m[0] != nullptr) {
    const float k_wr = 4.0f *
                       (robot_.chassis()->param().halfWheelBase +
                        robot_.chassis()->param().halfTrack) /
                       robot_.chassis()->param().wheelRadius;
    gimbal_ff_raw = m[0]->torq2Raw(kGimbalTqGain * gimbal_tq_lpf_ / k_wr);
  }

  int i;
  for (i=0; i<4; i++){
    float fb = m[i]->vel();
    if (i==1 || i == 2){
      fb = -fb;
    }
    if (m[i] != nullptr && p[i] != nullptr) {
    const float tgt = robot_.chassis()->wheelSpeeds()[i];
    float out = p[i]->Calc(tgt, fb);

    out += stictionFF(tgt);

    out += ((i == 1 || i == 2) ? -gimbal_ff_raw : gimbal_ff_raw);
    if (i==1 || i==2){
      out = -out;
    }
    m[i]->setInput(out);
    }
  }

if (robot_.getrc()->new_frame()) {

  volatile float pitch_delta = robot_.getrc()->rc_rv();
  pitch_delta *= kMousePitchSens;
  float yaw_delta = 0.0f;
  if (robot_.chassisMode() == robot::ChassisMode::Separate) {
    yaw_delta = 0.0f;
  } else{
  yaw_delta = robot_.getrc()->rc_rh() * kMouseYawSens;
  }

  robot_.comm()->chassis_data().pitch_ref_deg += pitch_delta;
  robot_.comm()->chassis_data().yaw_ref_deg += yaw_delta;

  robot_.comm()->chassis_data().pitch_ref_deg = clamp(robot_.comm()->chassis_data().pitch_ref_deg, kPitchMinDeg, kPitchMaxDeg);
  robot_.comm()->chassis_data().yaw_ref_deg = wrapTo180(robot_.comm()->chassis_data().yaw_ref_deg);

  robot_.getrc()->clear_new_frame();
}

last_chassis_mode_ = now_chassis_mode_;
last_working_ = true;
first_working_ = false;
}

else {
  for (int i = 0; i < 4; ++i) {
    if (m[i]) m[i]->setInput(0.0f);
  }
    freezeRefToGimbalFeedback();

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
extern "C" {
volatile bool g_tick_1ms = false;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {

  if (htim->Instance == TIM6) {
    g_tick_1ms = true;
  }
}
