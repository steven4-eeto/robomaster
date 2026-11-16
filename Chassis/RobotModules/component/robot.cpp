/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:36:50
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-08 20:34:59
 * @FilePath: \robomaster\Chassis\RobotModules\component\robot.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include <chassis.hpp>
#include <gimbal_chassis_comm.hpp>
#include "motor_base.hpp"
#include "DT7.hpp"

#include "robot.hpp"

#include <cstddef>
#include <cstdint>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

constexpr std::uint32_t kResurrectionTimeoutTicks = 3000;

constexpr float kSpinWz = 10.0f;

constexpr float kFollowYawKp = 1.0f;

constexpr float kMaxVx = 2.0f;
constexpr float kMaxVy = 2.0f;
constexpr float kMaxW  = 3.0f;
}

/* Private types -------------------------------------------------------------*/
namespace robot {

class DeadState : public LifeStateBase {
 public:
  static DeadState& instance();

  LifeState type() const override { return LifeState::kDead; }
  void onEnter(Robot& robot) override;
  void onExit(Robot& robot) override;
  LifeStateBase* update(Robot& robot) override;

 private:
  DeadState() = default;
  ~DeadState() override = default;
  DeadState(const DeadState&)            = delete;
  DeadState& operator=(const DeadState&) = delete;
};

class ResurrectionState : public LifeStateBase {
 public:
  static ResurrectionState& instance();

  LifeState type() const override { return LifeState::kResurrection; }
  void onEnter(Robot& robot) override;
  void onExit(Robot& robot) override;
  LifeStateBase* update(Robot& robot) override;

 private:
  ResurrectionState() = default;
  ~ResurrectionState() override = default;
  ResurrectionState(const ResurrectionState&)            = delete;
  ResurrectionState& operator=(const ResurrectionState&) = delete;

  std::uint32_t ticks_ = 0;
};

class WorkingState : public LifeStateBase {
 public:
  static WorkingState& instance();

  LifeState type() const override { return LifeState::kWorking; }
  void onEnter(Robot& robot) override;
  void onExit(Robot& robot) override;
  LifeStateBase* update(Robot& robot) override;

 private:
  WorkingState() = default;
  ~WorkingState() override = default;
  WorkingState(const WorkingState&)            = delete;
  WorkingState& operator=(const WorkingState&) = delete;
};

DeadState& DeadState::instance() {
  static DeadState s;
  return s;
}

void ResurrectionState::onEnter(Robot& robot) {
  ticks_ = 0;
}

void ResurrectionState::onExit(Robot& robot) {
}

ResurrectionState& ResurrectionState::instance() {
  static ResurrectionState s;
  return s;
}

WorkingState& WorkingState::instance() {
  static WorkingState s;
  return s;
}

void DeadState::onEnter(Robot& /*robot*/) {
}

void DeadState::onExit(Robot& /*robot*/) {}

LifeStateBase* DeadState::update(Robot& robot) {

  if (robot.chassis() == nullptr) {
    return nullptr;
  }
  if (robot.imu() == nullptr) {
    return nullptr;
  }
  if (robot.comm() == nullptr) {
    return nullptr;
  }
  for (std::size_t i = 0; i < Robot::kWheelNum; ++i) {
    if (robot.wheelMotor(i) == nullptr) {
      return nullptr;
    }
  }

  return &ResurrectionState::instance();
}

LifeStateBase* ResurrectionState::update(Robot& robot) {
  ++ticks_;

  bool imu_ready = true;
  if (robot.imu() != nullptr) {
    imu_ready = robot.imu()->isCalibrated();
  }

  if (imu_ready) {
    return &WorkingState::instance();
  }
  if (ticks_ >= kResurrectionTimeoutTicks) {
    return &DeadState::instance();
  }
  return nullptr;
}

void WorkingState::onEnter(Robot&) {}

void WorkingState::onExit(Robot&) {}

LifeStateBase* WorkingState::update(Robot& robot) {

  if (robot.chassis() == nullptr || robot.imu() == nullptr ||
      robot.comm() == nullptr) {
    return &DeadState::instance();
  }
  for (std::size_t i = 0; i < Robot::kWheelNum; ++i) {
    if (robot.wheelMotor(i) == nullptr) {
      return &DeadState::instance();
    }
  }
  return nullptr;
}

void Robot::processStateMachine() {

  if ((state_ == nullptr) && (next_state_ == nullptr)) {
    next_state_ = &DeadState::instance();
  }

  if ((next_state_ != nullptr) && (next_state_ != state_)) {
    if (state_ != nullptr) {
      state_->onExit(*this);
    }

    resetChassisCommand();
    resetComm();

    next_state_->onEnter(*this);
    state_          = next_state_;
    life_state_enum_ = state_->type();
    next_state_     = nullptr;
  } else if (next_state_ == state_) {
    next_state_ = nullptr;
  }

  if (state_ != nullptr) {
    LifeStateBase* next = state_->update(*this);
    if (next != nullptr) {
      next_state_ = next;
    }
  }
}
//-------------------------------------------------
void Robot::resetChassisCommand() {
  if (chassis_ != nullptr) {
    chassis_->setVelocity(0.0f, 0.0f, 0.0f);
  }
}

void Robot::resetComm() {
  if (comm_ != nullptr) {
    comm_->csetready(0.0f);
  }
}
//----------------------------------------------------
Robot::Robot() = default;

Robot::~Robot() = default;

void Robot::registerChassis(robot::Chassis* chassis) {
  chassis_ = chassis;
}

void Robot::registerImu(hello_world::imu::Imu* imu) {
  imu_ = imu;
}

void Robot::registerWheelMotor(std::size_t idx, hello_world::motor::Motor* motor) {
  if (idx < kWheelNum) {
    wheel_motors_[idx] = motor;
  }
}

void Robot::registerWheelMotors(hello_world::motor::Motor* m0, hello_world::motor::Motor* m1,
                                hello_world::motor::Motor* m2, hello_world::motor::Motor* m3) {
  wheel_motors_ = {m0, m1, m2, m3};
}

void Robot::registerComm(robot::GimbalChassisComm* comm) {
  comm_ = comm;
}

void Robot::registerDT7(hello_world::remote_control::DT7* Dt7_){
  rc_ = Dt7_;
}

void Robot::setEmergencyStop(bool on) {
  emergency_stop_ = on;
  if (on) {
    resetChassisCommand();
    resetComm();
  }
}
//--------------------------------------------------
extern Robot robot_;

}

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
