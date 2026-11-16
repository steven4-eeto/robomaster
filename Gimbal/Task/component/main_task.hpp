/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-09-19 15:12:00
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-08 08:22:17
 * @FilePath: \robomaster\Gimbal\Task\component\main_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :main_task.hpp
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
#ifndef GIMBAL_TASK_MAIN_TASK_HPP_
#define GIMBAL_TASK_MAIN_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool g_tick_1ms;

/* Exported function prototypes ----------------------------------------------*/
void MainTaskInit(void);
void MainTask(void);
#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TASK_MAIN_TASK_HPP_ */
