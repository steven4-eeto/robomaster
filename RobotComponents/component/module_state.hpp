/*
 * @Author: yinxingwen 3481976090@qq.com
 * @Date: 2026-10-07 13:04:26
 * @LastEditors: yinxingwen 3481976090@qq.com
 * @LastEditTime: 2026-10-27 18:51:04
 * @FilePath: \robomaster\RobotComponents\component\module_state.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :module_state.hpp
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
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
namespace robot {

/* ============================================================================
 * 【新人任务】定义底盘板与云台板共享的状态类型与枚举（namespace robot）。
 *
 * 本文件会被底盘板、云台板同时编译，只能放「纯类型 / 枚举 / 内联函数」，
 * 严禁依赖任何具体硬件（电机 / CAN / HAL），否则另一块板编不过。
 *
 * 你要做：
 *   1. 生命状态枚举：至少区分「上电未就绪 / 恢复自检中 / 正常工作中」三种，
 *      供 RobotModules 的状态机在 Dead / Resurrection / Working 三态间迁移使用。
 *   2. 手动控制源枚举：遥控器 / 键鼠（如有），说明当前控制指令来源。
 *   3. 底盘工作模式枚举：分离 / 随动 / 小陀螺；云台工作模式枚举（关节空间控 yaw、pitch）。
 *   4. 底盘运动状态结构（平移速度 vx、vy + 旋转角速度 w，单位自定并全工程一致）与
 *      云台运动状态结构（yaw、pitch），并给出加减、数乘等常用运算。
 *
 * 约束 / 提示：
 *   - 这些类型会被 RobotModules 状态机、RobotComponents 双板通讯帧、Instance 接线
 *     三处共同使用，字段命名 / 枚举值必须三处一致——动手前先画一张字段对照表。
 *   - 只需覆盖本骨架用到的状态（裁判系统 / 摩擦轮 / 超电相关状态本骨架用不到，可省略）
 * ==========================================================================*/
enum class LifeState : uint8_t {
    kDead         = 0,
    kResurrection = 1,
    kWorking      = 2,
};

enum class ManualSource : uint8_t {
    None             = 0,
    RemoteController = 1,//通过遥控器控制
    //预留
};

enum class ChassisMode : uint8_t {
    Separate = 0,
    Follow   = 1,
    Spin     = 2,
};

enum class GimbalMode : uint8_t {
    Joint = 0,//yaw，pitch独立
    //预留；
};

}

 #endif /* MODULE_STATE_HPP_ */
