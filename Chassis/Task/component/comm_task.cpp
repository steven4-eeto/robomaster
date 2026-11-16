/*
 * @Author: miunoki 13567781025@163.com
 * @Date: 2026-09-19 15:11:39
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-11-10 18:40:09
 * @FilePath: \omni_skeleton\Chassis\Task\component\comm_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"
#include "HW_can.hpp"
#include "iwdg.h"
#include "DT7.hpp"
#include "gimbal_chassis_comm.hpp"
#include "robot.hpp"

extern robot::GimbalChassisComm comm_;
extern robot::Robot robot_;
extern hello_world::remote_control::DT7 rc_;
extern hello_world::motor::Motor *m1;
extern hello_world::motor::Motor *m2;
extern hello_world::motor::Motor *m3;
extern hello_world::motor::Motor *m4;

extern "C" {
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
}

namespace {
constexpr uint16_t kDbusFrameSize = 18U;
uint8_t g_dbus_rx_buf[kDbusFrameSize] = {0};
}

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   1. 初始化 CAN 滤波器：inc/HW_can.hpp 的 CanFilter_Init(&hcan1) / CanFilter_Init(&hcan2)。
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */
  robot_.registerDT7(&rc_);
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) Error_Handler();

  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf, kDbusFrameSize);

  (void)HAL_IWDG_Refresh(&hiwdg);
}

void CommTask(void) {
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   1. 遍历电机：调用 encode(data) 得到 8 字节 + tx_id()，再 CAN_Send_Msg 发出。
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */
  uint8_t data[8] = {0};

  for (auto &motor : robot_.getmotors()) {
    if (motor == nullptr) continue;

    motor->encode(data);
  }

    (void)CAN_Send_Msg(&hcan2, data, 0x200, 8U);

  uint8_t dual_board_data[8] = {0};
  comm_.encode(dual_board_data);
  (void)CAN_Send_Msg(&hcan1, dual_board_data, comm_.tx_id(), 8U);
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart == &huart3) {
    if (Size == kDbusFrameSize) {
      rc_.decode(g_dbus_rx_buf, HAL_GetTick());
    }
    (void)HAL_IWDG_Refresh(&hiwdg);

    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, g_dbus_rx_buf,
                                       kDbusFrameSize);
  }
}
