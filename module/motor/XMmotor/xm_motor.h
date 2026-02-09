/**
 * @file xm_motor.h
 * @brief 小米电机驱动头文件 (XiaoMi/CyberGear Motor)
 * @note  基于CAN 2.0扩展帧协议,波特率1Mbps
 *        目前只实现运控模式(MIT模式)
 * 
 * @usage 使用示例:
 *        // 1. 定义电机初始化配置
 *        Motor_Init_Config_s xm_config = {
 *            .can_init_config = {
 *                .can_handle = &hfdcan1,
 *                .rx_id = 1,  // 电机ID (1~127)
 *            },
 *            .controller_param_init_config = {
 *                .angle_PID = { .Kp = 4.0f },  // MIT模式位置Kp
 *                .speed_PID = { .Kd = 0.2f },   // MIT模式阻尼Kd
 *            },
 *            .controller_setting_init_config = {
 *                .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
 *            },
 *        };
 *        
 *        // 2. 初始化电机 (注意小米电机上电1s之后再发指令)
 *        XMMotorInstance *xm_motor = XMMotorInit(&xm_config);
 *        
 *        // 3. 初始化电机控制任务 (在所有电机初始化完成后调用)
 *        XMMotorControlInit();
 *        
 *        // 4. 设置电机运控参数 (位置rad, 速度rad/s, 力矩Nm)
 *        XMMotorSetRef(xm_motor, 0.0f, 5.0f, 0.0f);  // 纯速度控制
 *        XMMotorSetRef(xm_motor, 1.0f, 0.0f, 0.0f);  // 位置控制
 *        XMMotorSetRef(xm_motor, 0.0f, 0.0f, 2.0f);  // 力矩控制
 *        XMMotorSetRef(xm_motor, 1.0f, 5.0f, 0.5f);  // 混合控制
 */

#ifndef XM_MOTOR_H
#define XM_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"

#define XM_MOTOR_CNT 4

/* 运控模式参数范围 */
#define XM_P_MIN  (-12.5f)
#define XM_P_MAX  12.5f
#define XM_V_MIN  (-30.0f)
#define XM_V_MAX  30.0f
#define XM_T_MIN  (-12.0f)
#define XM_T_MAX  12.0f
#define XM_KP_MIN 0.0f
#define XM_KP_MAX 500.0f
#define XM_KD_MIN 0.0f
#define XM_KD_MAX 5.0f

/* 主机CAN ID (用于标识主控制器) */
#define XM_MASTER_CAN_ID 0x7F

/**
 * @brief 小米电机CAN扩展帧ID结构体
 * @note  29位扩展ID格式:
 *        Bit28~24: 通信类型 (5bits)
 *        Bit23~8:  数据区2 (16bits) - 主机CAN_ID/预设参数等
 *        Bit7~0:   目标电机CAN_ID (8bits)
 */
typedef struct
{
    uint32_t id : 8;      // 目标电机CAN_ID, bit7~0
    uint32_t data : 16;   // 数据区2, bit23~8
    uint32_t mode : 5;    // 通信类型, bit28~24
    uint32_t res : 3;     // 保留位, bit31~29
} XM_CanIdInfo_s;

/**
 * @brief 通信类型枚举
 */
typedef enum
{
    XM_CMD_GET_DEVICE_ID = 0,     // 通信类型0: 获取设备ID
    XM_CMD_CONTROL_MODE = 1,      // 通信类型1: 运控模式电机控制指令
    XM_CMD_MOTOR_FEEDBACK = 2,    // 通信类型2: 电机反馈数据
    XM_CMD_MOTOR_ENABLE = 3,      // 通信类型3: 电机使能运行
    XM_CMD_MOTOR_STOP = 4,        // 通信类型4: 电机停止运行
    XM_CMD_SET_ZERO = 6,          // 通信类型6: 设置电机机械零位
    XM_CMD_SET_CAN_ID = 7,        // 通信类型7: 设置电机CAN_ID
    XM_CMD_READ_PARAM = 17,       // 通信类型17: 单个参数读取
    XM_CMD_WRITE_PARAM = 18,      // 通信类型18: 单个参数写入
    XM_CMD_FAULT_FEEDBACK = 21,   // 通信类型21: 故障反馈帧
    XM_CMD_SET_BAUDRATE = 22,     // 通信类型22: 波特率修改
} XM_Comm_Type_e;

/**
 * @brief 电机模式状态枚举 (bit22~23)
 */
typedef enum
{
    XM_MODE_RESET = 0,    // Reset模式 [复位]
    XM_MODE_CALI = 1,     // Cali模式 [标定]
    XM_MODE_MOTOR = 2,    // Motor模式 [运行]
} XM_Motor_Mode_State_e;

/**
 * @brief 电机故障状态位定义 (bit21~16)
 *        0=无故障, 1=有故障
 */
typedef enum
{
    XM_FAULT_NONE = 0,
    XM_FAULT_UNCALIBRATED = (1 << 5),   // bit21: 未标定
    XM_FAULT_HALL_ENCODE = (1 << 4),    // bit20: HALL编码故障
    XM_FAULT_MAG_ENCODE = (1 << 3),     // bit19: 磁编码故障
    XM_FAULT_OVER_TEMP = (1 << 2),      // bit18: 过温
    XM_FAULT_OVER_CURRENT = (1 << 1),   // bit17: 过流
    XM_FAULT_UNDER_VOLTAGE = (1 << 0),  // bit16: 欠压故障
} XM_Fault_Type_e;

/**
 * @brief 小米电机CAN反馈信息结构体
 */
typedef struct
{
    uint8_t id;                     // 电机CAN ID
    uint8_t state;                  // 电机状态 (模式+故障)
    XM_Motor_Mode_State_e mode;     // 电机模式状态
    uint8_t fault;                  // 故障状态
    float position;                 // 当前角度, 对应(-4π ~ 4π)
    float velocity;                 // 当前角速度, 对应(-30rad/s ~ 30rad/s)
    float last_position;            // 上一次角度值
    float torque;                   // 当前力矩, 对应(-12Nm ~ 12Nm)
    float temperature;              // 当前温度, 摄氏度
    int32_t total_round;            // 总圈数(累计)
} XM_Motor_Measure_s;

/**
 * @brief 小米电机CAN发送信息结构体 (运控模式)
 */
typedef struct
{
    uint16_t position_des;  // 目标角度 [0~65535] -> (-4π ~ 4π)
    uint16_t velocity_des;  // 目标角速度 [0~65535] -> (-30rad/s ~ 30rad/s)
    uint16_t Kp;            // Kp [0~65535] -> (0 ~ 500)
    uint16_t Kd;            // Kd [0~65535] -> (0 ~ 5)
    uint16_t torque_des;    // 力矩 [0~65535] -> (-12Nm ~ 12Nm)
} XM_Motor_Send_s;

/**
 * @brief 小米电机实例结构体
 */
typedef struct
{
    XM_Motor_Measure_s measure;             // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机控制设置
    
    float Kp;                               // MIT模式位置Kp
    float Kd;                               // MIT模式速度Kd
    
    float *other_angle_feedback_ptr;        // 其他角度反馈来源指针
    float *other_speed_feedback_ptr;        // 其他速度反馈来源指针
    float *speed_feedforward_ptr;           // 速度前馈指针
    float *current_feedforward_ptr;         // 电流前馈指针
    
    float position_ref;                     // 位置参考值 (rad)
    float velocity_ref;                     // 速度参考值 (rad/s)
    float torque_ref;                       // 力矩参考值 (Nm)
    
    Motor_Working_Type_e stop_flag;         // 启停标志
    
    CANInstance *motor_can_instance;        // CAN实例
    
    uint8_t motor_id;                       // 电机CAN ID (1~127)
    uint16_t master_id;                     // 主机CAN ID
    
    uint32_t lost_cnt;                      // 丢包计数
} XMMotorInstance;

/**
 * @brief 初始化小米电机
 * @param config 电机初始化配置结构体指针
 * @return XMMotorInstance* 电机实例指针
 */
XMMotorInstance *XMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置电机运控模式参考值 (同时设置位置、速度、力矩)
 * @param motor 电机实例指针
 * @param position 目标位置 (rad), 范围: -12.5 ~ 12.5
 * @param velocity 目标速度 (rad/s), 范围: -30 ~ 30
 * @param torque 前馈力矩 (Nm), 范围: -12 ~ 12
 * @note  运控模式公式: τ = Kp*(pos_ref - pos) + Kd*(vel_ref - vel) + torque_ref
 */
void XMMotorSetRef(XMMotorInstance *motor, float position, float velocity, float torque);

/**
 * @brief 设置电机外环类型
 * @param motor 电机实例指针
 * @param closeloop_type 闭环类型
 */
void XMMotorOuterLoop(XMMotorInstance *motor, Closeloop_Type_e closeloop_type);

/**
 * @brief 使能电机
 * @param motor 电机实例指针
 */
void XMMotorEnable(XMMotorInstance *motor);

/**
 * @brief 停止电机
 * @param motor 电机实例指针
 */
void XMMotorStop(XMMotorInstance *motor);

/**
 * @brief 设置电机机械零位
 * @param motor 电机实例指针
 */
void XMMotorCaliEncoder(XMMotorInstance *motor);

/**
 * @brief 初始化电机控制任务
 * @note  在所有电机实例初始化完成后调用
 */
void XMMotorControlInit(void);

#endif // XM_MOTOR_H
