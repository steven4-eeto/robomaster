/**
 *******************************************************************************
 * @file      :robot.hpp
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
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <array>
#include <cstddef>
#include <cstdint>

#include <module_state.hpp>
#include <imu.hpp>
#include <ins_motor.hpp>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot { class Chassis; }
namespace hello_world {
namespace imu {
    class Imu; 
}
}
namespace hello_world {
namespace motor   { class Motor; } }
namespace robot    { class GimbalChassisComm; }
namespace hello_world {
namespace remote_control { class DT7;
}}

namespace robot {

class Robot;

class LifeStateBase {
 public:
  virtual ~LifeStateBase() = default;

  virtual LifeState type() const = 0;

  virtual void onEnter(Robot& robot) = 0;

  virtual void onExit(Robot& robot) = 0;

  virtual LifeStateBase* update(Robot& robot) = 0;
};

class DeadState;
class ResurrectionState;
class WorkingState;

class Robot {
 public:
  static constexpr std::size_t kWheelNum  = 4;
  static constexpr float       kPeriodSec = 0.001f;

  Robot();
  ~Robot();

  Robot(const Robot&)            = delete;
  Robot& operator=(const Robot&) = delete;

  void registerChassis(robot::Chassis* chassis);
  void registerImu(hello_world::imu::Imu* imu);
  void registerWheelMotor(std::size_t idx, hello_world::motor::Motor* motor);
  void registerWheelMotors(hello_world::motor::Motor* m0, hello_world::motor::Motor* m1,
                           hello_world::motor::Motor* m2, hello_world::motor::Motor* m3);
  void registerComm(robot::GimbalChassisComm* comm);
  void registerDT7(hello_world::remote_control::DT7* Dt7_);

  void update();

  LifeState     lifeState()     const { return life_state_enum_; }
  ChassisMode      chassisMode()      const { return chassis_mode_; }
  void setchassismode(robot::ChassisMode cm_){chassis_mode_ = cm_;} 
  ManualSource manualSource() const { return manual_source_; }
  bool          emergencyStop() const { return emergency_stop_; }

  void setWorkMode(ChassisMode mode)            { chassis_mode_ = mode; }
  void setControlSource(ManualSource src)   { manual_source_ = src; }
  void setEmergencyStop(bool on);

  void requestTransition(LifeStateBase* next) { next_state_ = next; }

  robot::Chassis*        chassis()  { return chassis_; }
  hello_world::imu::Imu*                imu()      { return imu_; }
  hello_world::motor::Motor* wheelMotor(std::size_t idx){ return GetMotorWheel(idx); }
  std::array<hello_world::motor::Motor*, 4> getmotors(){ return wheel_motors_; }
  void setmotors(hello_world::motor::Motor* m[4]){
    for (size_t i = 0; i < 4; ++i) {
        wheel_motors_[i] = m[i];
    }
  }
  robot::GimbalChassisComm* comm()     { return comm_; }

  hello_world::remote_control::DT7* getrc(){return rc_;}
  LifeState getstate(){return life_state_enum_;}

 public:

  void updateInputs();
  void processStateMachine();
  void updateControl();

  void resetChassisCommand();
  void resetComm();
  
 private:

  robot::Chassis*        chassis_ = nullptr;
  hello_world::imu::Imu*                imu_     = nullptr;
  std::array<hello_world::motor::Motor*, kWheelNum> wheel_motors_{};
  robot::GimbalChassisComm* comm_    = nullptr;
  hello_world::remote_control::DT7* rc_ = nullptr;

  LifeStateBase* state_          = nullptr;  ///< 当前状态
  LifeStateBase* next_state_     = nullptr;  ///< 待迁移状态
  LifeState      life_state_enum_ = LifeState::kDead;

  ChassisMode      chassis_mode_      = ChassisMode::Separate;
  ManualSource manual_source_ = ManualSource::None;
  bool          emergency_stop_ = false;
};

}
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】声明底盘板顶层状态机类 Robot（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Robot 类，作为本板每个控制周期（1ms）的入口：汇总各模块数据，
 *      生成控制指令并下发给执行部件。
 *   2. 实现「生命状态」三态切换：Dead（上电未就绪 / 离线）→ Resurrection（恢复
 *      自检）→ Working（正常控制），并定义各状态进入 / 退出时的复位动作。
 *   3. 在 Working 态下按手动控制源与工作模式（分离 / 随动 / 小陀螺）生成底盘运动指令。
 *   4. 提供注册各模块指针的接口（Chassis、Imu、4 轮电机、GimbalChassisComm），并在每个周期按正确顺序更新数据。
 *
 * 约束 / 提示：
 *   - 底盘板与云台板之间只能通过 GimbalChassisComm 的
 *     CAN 帧交换信息，不能直接互相调用函数或共享变量。
 *   - 控制周期入口由 Task/component/main_task.cpp 的定时中断调用，本类自己不要开中断、
 *     不要死等延时。
 *   - 急停：遥控器拨到急停位时，必须保证所有电机指令归零（硬性验收项）。
 *   - 状态机的基类 / 迁移机制要自己设计，本骨架不提供（裁判系统 / 超电 / 蜂鸣器等组件本骨架也没有，可忽略）。
 * ==========================================================================*/
#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
