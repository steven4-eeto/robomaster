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

#include "main.h"
#include "HW_can.hpp"
#include "DT7.hpp"
#include "iwdg.h"
#include "ins_motor.hpp"
#include "gimbal_chassis_comm.hpp"
#include "robot.hpp"

#define DBUS_RX_SIZE 18u

#define DBUS_UART_HANDLE huart3

extern hello_world::remote_control::DT7 rc_;

extern hello_world::motor::DM_J4310* yaw_motor_;
extern hello_world::motor::DM_J4310* pitch_motor_;

extern robot::GimbalChassisComm comm_;

extern robot::Robot robot_;

static uint8_t s_dbus_rx_buf[DBUS_RX_SIZE];

extern "C" {
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
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

  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) Error_Handler();

  (void)HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_UART_HANDLE, s_dbus_rx_buf,
                                     DBUS_RX_SIZE);

  (void)HAL_IWDG_Refresh(&hiwdg);
}

void CommTask(void) {
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   1. 遍历电机：调用 encode(data) 得到 8 字节 + tx_id()，再 CAN_Send_Msg 发出。
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */
  uint8_t data[8];

    yaw_motor_->encode(data);
    (void)CAN_Send_Msg(&hcan1, data, yaw_motor_->tx_id(), sizeof(data));
    //uint8_t data_[8] = {0};
    //(void)CAN_Send_Msg(&hcan1, data_, yaw_motor_->tx_id(), sizeof(data));

    pitch_motor_->encode(data);
    (void)CAN_Send_Msg(&hcan2, data, pitch_motor_->tx_id(), sizeof(data));

  robot_.getGimbalChassisComm()->encode(data);
  (void)CAN_Send_Msg(&hcan1, data, robot_.getGimbalChassisComm()->tx_id(), sizeof(data));

  (void)HAL_IWDG_Refresh(&hiwdg);

}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
