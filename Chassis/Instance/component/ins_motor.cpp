/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-10-28 09:21:50
 * @FilePath: \robomaster\Chassis\Instance\component\ins_motor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
#include "motor_M3508.hpp"
#include "motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号轮电机（M3508）单例。
   *
   * 你要做：
   *   1. 静态定义 4 个轮电机对象并存入数组（可用 hello_world::motor::CreateMotor
   *      (hello_world::motor::MotorType::kM3508, id, opt) 工厂，或直接构造 M3508）。
   *   2. 校验 idx ∈ [0,4)，返回对应元素的地址。
   *   3. 每个电机的 CAN id、方向、减速比按实机接线配置（见考核说明文档）。
   *
   * 约束 / 提示：
   *   - 电机设备类已提供：inc/motor_M3508.hpp（M3508）、inc/motor.hpp（CreateMotor 工厂、
   *     OptionalParams 配置项）。
   */
  using hello_world::motor::CreateMotor;
  using hello_world::motor::Dir;
  using hello_world::motor::Motor;
  using hello_world::motor::MotorType;
  using hello_world::motor::OptionalParams;
  using hello_world::motor::kDirFwd;
  using hello_world::motor::kDirRev;

  if (idx < 0 || idx >= 4) {
    return nullptr;
  }

  static const int kCanId[4] = {1, 2, 3, 4};
  static const Dir kDir[4] = {kDirRev, kDirRev, kDirRev, kDirRev};
  static const float kReductionRatio[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  auto make_opt = [](int i) -> OptionalParams {
    OptionalParams opt;
    opt.dir = kDir[i];
    opt.ex_redu_rat = kReductionRatio[i];
    return opt;
  };

  static Motor *const motors[4] = {
      CreateMotor(MotorType::kM3508, kCanId[0], make_opt(0)),
      CreateMotor(MotorType::kM3508, kCanId[1], make_opt(1)),
      CreateMotor(MotorType::kM3508, kCanId[2], make_opt(2)),
      CreateMotor(MotorType::kM3508, kCanId[3], make_opt(3)),
  };

  return motors[idx];

  //(void)idx;
  //return nullptr;
}

extern hello_world::motor::Motor *m1;
extern hello_world::motor::Motor *m2;
extern hello_world::motor::Motor *m3;
extern hello_world::motor::Motor *m4;

/* Private function definitions ----------------------------------------------*/
