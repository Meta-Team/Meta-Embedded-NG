#ifndef REFEREE_TASK_26_H
#define REFEREE_TASK_26_H

#include "referee_26.h"
#include "robot_def.h"

#pragma pack(1)

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t gimbal_flag : 1;
    uint32_t shoot_flag : 1;
    uint32_t lid_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    chassis_mode_e chassis_mode;             // 底盘模式
    gimbal_mode_e gimbal_mode;               // 云台模式
    shoot_mode_e shoot_mode;                 // 发射模式设置
    friction_mode_e friction_mode;           // 摩擦轮关闭
    lid_mode_e lid_mode;                     // 弹舱盖打开
    Chassis_Power_Data_s Chassis_Power_Data; // 功率控制

    // 上一次的模式，用于flag判断
    chassis_mode_e chassis_last_mode;
    gimbal_mode_e gimbal_last_mode;
    shoot_mode_e shoot_last_mode;
    friction_mode_e friction_last_mode;
    lid_mode_e lid_last_mode;
    Chassis_Power_Data_s Chassis_last_Power_Data;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();

#endif // REFEREE_TASK_26_H
