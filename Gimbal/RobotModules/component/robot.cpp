/**
 *******************************************************************************
 * @file      :robot.cpp
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
#include "robot.hpp"
#include "imu.hpp"
#include "gimbal.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】实现 robot.hpp 中声明的状态机。
 *
 * 你要做：
 *   1. 周期入口：按「更新数据 → 状态机迁移 → 生成指令 → 下发」的顺序组织 update()。
 *   2. 三态实现：Dead 时输出全零并等待就绪条件；Resurrection 时做自检与复位（如等待
 *      电机使能、IMU 零漂收敛）；Working 时执行正常控制逻辑。
 *   3. 各工作模式（分离 / 随动 / 小陀螺）的指令生成分支。
 *
 * 约束 / 提示：
 *   - 状态切换时记得复位上一状态残留的指令，避免电机突变。
 *   - 所有下发指令统一经过本类，急停判断放在最外层，确保任何分支都先过急停。
 * ==========================================================================*/
namespace robot {

void Robot::init()
{
  gimbal_ = nullptr;
  imu_ = nullptr;
  yaw_motor_ = nullptr;
  pitch_motor_ = nullptr;
  comm_ = nullptr;

  life_state_ = LifeState::kDead;
  resurrection_done_ = false;
  emergency_stop_ = false;

  resetTargets();
  resetControlCommand();

  onEnterDead();
}

void Robot::update()
{

  updateFeedback();
  checkEmergencyStop();
  updateTargets();
  updateLifeStateMachine();
  updateControl();
  updateCommunicationAndOutput();
}

void Robot::registerGimbal(Gimbal* gimbal)
{
  gimbal_ = gimbal;
}

void Robot::registerImu(hello_world::imu::Imu* imu)
{
  imu_ = imu;
}

void Robot::registerYawMotor(hello_world::motor::Motor* yaw_motor)
{
  yaw_motor_ = yaw_motor;
}

void Robot::registerPitchMotor(hello_world::motor::Motor* pitch_motor)
{
  pitch_motor_ = pitch_motor;
}

void Robot::registerGimbalChassisComm(GimbalChassisComm* comm)
{
  comm_ = comm;
}

void Robot::registerDT7(hello_world::remote_control::DT7* rc){
    rc_ = rc;
}

void Robot::setRemoteTarget(float yaw_rad, float pitch_rad)
{
  remote_target_.yaw_rad = yaw_rad;
  remote_target_.pitch_rad = pitch_rad;
  remote_target_.valid = true;

}

void Robot::setChassisTarget(float yaw_rad, float pitch_rad)
{
  chassis_target_.yaw_rad = yaw_rad;
  chassis_target_.pitch_rad = pitch_rad;
  chassis_target_.valid = true;

}

void Robot::setEmergencyStop(bool on)
{
  emergency_stop_ = on;

  if (emergency_stop_) {

    forceZeroOutputs();

    // TODO: 若底层电机接口允许，应在此处立即下发零电流/零输出。
  }
}

bool Robot::isEmergencyStop() const
{
  return emergency_stop_;
}

Robot::LifeState Robot::lifeState() const
{
  return life_state_;
}

const Robot::ControlCommand& Robot::controlCommand() const
{
  return control_command_;
}

bool Robot::modulesReady() const
{
  return gimbal_ != nullptr &&
         imu_ != nullptr &&
         yaw_motor_ != nullptr &&
         pitch_motor_ != nullptr &&
         comm_ != nullptr;
}

void Robot::checkEmergencyStop()
{
  if (emergency_stop_) {
    forceZeroOutputs();
  }

  // TODO: 实际工程中应在此处从遥控器/通讯反馈中读取急停开关状态，
  //       并调用 setEmergencyStop(...) 或直接更新 emergency_stop_。
}

void Robot::updateFeedback()
{
  // TODO: 按项目实际接口更新：
  // 1. imu_ 姿态、角速度、零漂收敛状态；
  // 2. yaw_motor_ / pitch_motor_ 角度、角速度、使能状态、故障状态；
  // 3. comm_ 中来自底盘板的期望 yaw/pitch、急停/模式信息；
  // 4. 遥控器/键鼠输入。
  //
  // 本骨架不直接解引用前置声明类型，避免因接口缺失导致编译失败。
}

void Robot::updateTargets()
{

  if (remote_target_.valid) {
    target_ = remote_target_;
  } else if (chassis_target_.valid) {
    target_ = chassis_target_;
  } else {
    target_ = Target{};
    target_.valid = false;
  }
}

void Robot::updateLifeStateMachine()
{
  switch (life_state_) {
    case LifeState::kDead: {

      if (!emergency_stop_ && modulesReady()) {
        transitionTo(LifeState::kResurrection);
      }
      break;
    }

    case LifeState::kResurrection: {

      if (emergency_stop_ || !modulesReady()) {
        transitionTo(LifeState::kDead);
      } else if (doResurrection() && imu_->isCalibrated()) {
        transitionTo(LifeState::kWorking);
        gimbal_->init();
      }
      break;
    }

    case LifeState::kWorking: {

      if (emergency_stop_ || !modulesReady()) {
        transitionTo(LifeState::kDead);
      }
      break;
    }

    default: {
      transitionTo(LifeState::kDead);
      break;
    }
  }
}

void Robot::updateControl()
{

  if (emergency_stop_) {
    forceZeroOutputs();
    return;
  }

  if (life_state_ != LifeState::kWorking) {
    forceZeroOutputs();
    return;
  }

  if (!target_.valid) {
    forceZeroOutputs();
    return;
  }

  // TODO: 此处应接入云台控制器/工作模式分支：
  // - 分离模式；
  // - 随动模式；
  // - 小陀螺模式。
  //
  // 由于当前骨架没有模式输入和 Gimbal 控制器接口，下面仅给出
  // “目标角 -> 指令”的占位输出。实际应替换为控制器输出。
  control_command_.yaw_output = target_.yaw_rad;
  control_command_.pitch_output = target_.pitch_rad;
  control_command_.output_enabled = true;
}

void Robot::updateCommunicationAndOutput()
{

  if (emergency_stop_ ||
      life_state_ != LifeState::kWorking ||
      !control_command_.output_enabled) {
    forceZeroOutputs();
  }

  // TODO: 按项目实际接口完成：
  // 1. 将 control_command_ 下发给 yaw_motor_ / pitch_motor_；
  // 2. 通过 comm_ 的 CAN 帧向底盘板发送云台状态/目标信息；
  // 3. 从 comm_ 接收底盘目标并调用 setChassisTarget(...)。
  //
  // 注意：与底盘板之间只能通过 GimbalChassisComm 的 CAN 帧交换信息，
  // 不能直接调用底盘板函数或共享变量。
}

void Robot::resetControlCommand()
{
  control_command_ = ControlCommand{};
}

void Robot::resetTargets()
{
  remote_target_ = Target{};
  chassis_target_ = Target{};
  target_ = Target{};
}

void Robot::forceZeroOutputs()
{
  control_command_.yaw_output = 0.0f;
  control_command_.pitch_output = 0.0f;
  control_command_.output_enabled = false;

  // TODO: 若底层要求“立即下发零输出”，应在此处调用电机/通讯下发接口。
  //       本类保证控制指令本身归零；实际硬件归零由出口适配层完成。
}

void Robot::transitionTo(LifeState next)
{
  if (next == life_state_) {
    return;
  }

  switch (life_state_) {
    case LifeState::kDead:
      onExitDead();
      break;
    case LifeState::kResurrection:
      onExitResurrection();
      break;
    case LifeState::kWorking:
      onExitWorking();
      break;
    default:
      break;
  }

  resetControlCommand();

  life_state_ = next;

  switch (life_state_) {
    case LifeState::kDead:
      onEnterDead();
      break;
    case LifeState::kResurrection:
      onEnterResurrection();
      break;
    case LifeState::kWorking:
      onEnterWorking();
      break;
    default:
      break;
  }
}

void Robot::onEnterDead()
{
  resurrection_done_ = false;
  forceZeroOutputs();
  resetControlCommand();
}

void Robot::onExitDead()
{
  resurrection_done_ = false;
}

void Robot::onEnterResurrection()
{
  resurrection_done_ = false;
  forceZeroOutputs();
  resetControlCommand();
}

void Robot::onExitResurrection()
{

}

void Robot::onEnterWorking()
{
  resurrection_done_ = true;
  resetControlCommand();
}

void Robot::onExitWorking()
{
  forceZeroOutputs();
  resetControlCommand();
}

bool Robot::doResurrection()
{
  if (emergency_stop_ || !modulesReady()) {
    return false;
  }

  // TODO: 实际复活自检应在此处完成，例如：
  // 1. 等待 yaw/pitch 电机使能成功；
  // 2. 等待 IMU 零漂收敛；
  // 3. 检查通讯链路正常；
  // 4. 检查目标数据有效。
  //
  // 当前骨架无具体模块接口，因此模块指针齐全且无急停时视为自检完成。
  resurrection_done_ = true;
  return true;
}

}
