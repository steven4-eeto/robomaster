/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:52:24
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-11 08:35:35
 * @FilePath: \robomaster\Gimbal\Instance\component\ins_motor.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_motor.hpp
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
#ifndef GIMBAL_INSTANCE_INS_MOTOR_HPP_
#define GIMBAL_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取云台 yaw 关节电机单例（DM-J4310）。 */
hello_world::motor::DM_J4310 *GetMotorYaw();
/* 获取云台 pitch 关节电机单例（DM-J4310）。 */
hello_world::motor::DM_J4310 *GetMotorPitch();

#endif /* GIMBAL_INSTANCE_INS_MOTOR_HPP_ */
