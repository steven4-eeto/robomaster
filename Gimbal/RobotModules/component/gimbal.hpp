/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-09 11:57:34
 * @FilePath: \robomaster\Gimbal\RobotModules\component\gimbal.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :gimbal.hpp
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
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_DaMiao.hpp"
#include "ins_pid.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】声明云台运动模块类 Gimbal（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Gimbal 类，实现 yaw / pitch 两个关节的关节空间位置控制（位置环）。
 *   2. 读入云台目标角度（来自遥控器 / 键鼠 / 底盘经双板通讯传来的期望值），输出
 *      yaw / pitch 电机的控制量。
 *   3. 处理 yaw 轴 360° 连续旋转的角度回绕，pitch 轴用软件限位防堵转。
 *
 * 约束 / 提示：
 *   - yaw / pitch 均为达妙 DM-J4310 电机，反馈角度 / 指令接口见 inc/motor_DaMiao.hpp。
 *   - 位置环 PID 由你自己实现（PID 类），本模块只负责调用。
 * ==========================================================================*/
namespace robot {

class Gimbal {
 public:
  Gimbal(hello_world::motor::DM_J4310* yaw_motor, hello_world::motor::DM_J4310* pitch_motor);
  ~Gimbal() = default;

  void init();

  void setTargetAngle(float yaw_ref, float pitch_ref);

  /* 设置 yaw 环的外部反馈（世界系，弧度，通常传 IMU yaw）。
   * 传入后，底盘旋转会被位置环自动补偿，云台保持对地不转；
   * 不调用则回退为电机编码器反馈（车体系，跟随底盘）。 */
  void setYawFeedback(float yaw_rad);

  void update(float dt);

  float yawAngle() const;
  float pitchAngle() const;
  float yawTarget() const;
  float pitchTarget() const;

  float getyawout(){return yaw_out;}
    float getpitchout(){return pitch_out;}

    void registerpid(hello_world::pid::Pid yaw_pid , hello_world::pid::Pid pitch_pid){
      yaw_pid_ = yaw_pid;
      pitch_pid_ = pitch_pid;
    }

 private:

  static float normalizeAngle(float angle);

  hello_world::motor::Motor* yaw_motor_{nullptr};
  hello_world::motor::Motor* pitch_motor_{nullptr};

  hello_world::pid::Pid yaw_pid_;
  hello_world::pid::Pid pitch_pid_;

  float yaw_ref_{0.0f};
  float pitch_ref_{0.0f};

  /* 外部 yaw 反馈（世界系）；valid 为 false 时用编码器反馈 */
  float yaw_fb_ext_{0.0f};
  bool  yaw_fb_ext_valid_{false};

  float yaw_out;
  float pitch_out;
};

}

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
