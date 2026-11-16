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
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】声明云台板顶层状态机类 Robot（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Robot 类，作为本板每个控制周期的入口：汇总云台目标角度（来自遥控器 /
 *      键鼠 / 底盘经双板通讯传来的期望值）、IMU 姿态、电机反馈，生成 yaw / pitch
 *      控制指令。
 *   2. 实现「生命状态」三态切换：Dead → Resurrection → Working，并定义各状态
 *      进入 / 退出时的复位动作。
 *   3. 提供注册各模块指针的接口（Gimbal、Imu、yaw / pitch 电机、GimbalChassisComm），
 *      并在每个周期按正确顺序更新数据。
 *
 * 约束 / 提示：
 *   - 与底盘板之间只能通过 GimbalChassisComm 的
 *     CAN 帧交换信息，不能直接互相调用函数或共享变量。
 *   - 控制周期入口由 Task/component/main_task.cpp 的定时中断调用，本类自己不要开中断、
 *     不要死等延时。
 *   - 急停：遥控器拨到急停位时，必须保证所有电机指令归零（硬性验收项）。
 *   - 状态机的基类 / 迁移机制要自己设计，本骨架不提供（裁判系统 / 超电 / 蜂鸣器等组件本骨架也没有，可忽略）。
 * ==========================================================================*/
namespace hello_world{
    namespace imu{
        class Imu;
    }
}

namespace hello_world{
    namespace motor{
        class Motor;
    }
}

namespace hello_world{
  namespace remote_control{
    class DT7;
  }
}

 namespace robot {

class Gimbal;

class GimbalChassisComm;

class Robot {
 public:

  enum class LifeState : uint8_t {
    kDead = 0,
    kResurrection = 1,
    kWorking = 2,
  };

  struct Target {
    float yaw_rad{0.0f};
    float pitch_rad{0.0f};
    bool valid{false};
    uint32_t stamp_ms{0};
  };

  struct ControlCommand {
    float yaw_output{0.0f};
    float pitch_output{0.0f};
    bool output_enabled{false};
  };

  Robot() = default;
  ~Robot() = default;

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  void init();

  void update();

  void registerGimbal(Gimbal* gimbal);
  Gimbal* getGimbal(){return gimbal_;}

  void registerImu(hello_world::imu::Imu* imu);
  hello_world::imu::Imu* getImu(){return imu_;}

  void registerYawMotor(hello_world::motor::Motor* yaw_motor);
  hello_world::motor::Motor* getYawMotor(){return yaw_motor_;}

  void registerPitchMotor(hello_world::motor::Motor* pitch_motor);
  hello_world::motor::Motor* getPitchMotor(){return pitch_motor_;}

  void registerGimbalChassisComm(GimbalChassisComm* comm);
  GimbalChassisComm* getGimbalChassisComm(){return comm_;}

  void registerDT7(hello_world::remote_control::DT7* rc);
  hello_world::remote_control::DT7* getDT7(){return rc_;}

  void setRemoteTarget(float yaw_rad, float pitch_rad);

  void setChassisTarget(float yaw_rad, float pitch_rad);

  void setEmergencyStop(bool on);

  bool isEmergencyStop() const;

  LifeState lifeState() const;

  const ControlCommand& controlCommand() const;

 public:

  bool modulesReady() const;

  void checkEmergencyStop();
  bool getemergencystop(){return emergency_stop_;}

  void updateFeedback();

  void updateTargets();

  void updateLifeStateMachine();

  void updateControl();

  void updateCommunicationAndOutput();

  void resetControlCommand();

  void resetTargets();

  void forceZeroOutputs();

  void transitionTo(LifeState next);
  LifeState getlifestate(){return life_state_;}
  void setlifestate(LifeState life_state){life_state_ = life_state;}

  void onEnterDead();

  void onExitDead();

  void onEnterResurrection();

  void onExitResurrection();

  void onEnterWorking();

  void onExitWorking();

  bool doResurrection();

 private:

  Gimbal* gimbal_{nullptr};
  hello_world::imu::Imu* imu_{nullptr};
  hello_world::motor::Motor* yaw_motor_{nullptr};
  hello_world::motor::Motor* pitch_motor_{nullptr};
  GimbalChassisComm* comm_{nullptr};
  hello_world::remote_control::DT7* rc_{nullptr};

  LifeState life_state_{LifeState::kDead};
  bool resurrection_done_{false};

  bool emergency_stop_{false};

  Target remote_target_{};
  Target chassis_target_{};
  Target target_{};

  ControlCommand control_command_{};
};

}

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
