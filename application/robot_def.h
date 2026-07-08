#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>

/*
 * 机器人种类选择器（唯一被共享模块引用的入口）。
 *
 * 本文件包含两部分：
 *   1. 通用消息契约：模块间通信用的枚举与结构体，与机器人种类无关，所有机器人共享。
 *   2. 参数选择：根据构建时宏引入对应机器人的专属物理参数头文件。
 *
 * 机器人种类由构建时宏决定：make ROBOT=sentry|infantry 会传入 -DROBOT_SENTRY / -DROBOT_INFANTRY。
 * 机器人专属的物理参数请放在各自的 <robot>_def.h 中。
 */

/* ============================ 通用消息契约 ============================ */

#pragma pack(1)
// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_ROTATE,            // 小陀螺模式
} Chassis_Mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_IMU = 0,           // IMU控制云台
    GIMBAL_LOCK,              // 云台电机强制锁定到0
} Gimbal_Mode_e;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分,采用底盘坐标系
    float vx;           // 前进方向速度 (m/s)
    float vy;           // 横移方向速度 (m/s)
    float wz;           // 旋转速度 (rad/s)
    Chassis_Mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

// chassis发布的底盘反馈数据,由cmd订阅
typedef struct
{
    float real_vx;
    float real_vy;
    float real_wz;
} Chassis_Upload_Data_s;

// cmd发布的底盘控制数据,由gimbal订阅
typedef struct
{
    // yaw, pitch的含义由模式决定
    float yaw;
    float pitch;
    Gimbal_Mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
    float yaw_deg;
    float yaw_speed;
} Gimbal_Upload_Data_s;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    // Bullet_Speed_e bullet_speed; // 弹速枚举
    // uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

#pragma pack()

/* ========================= 机器人专属参数选择 ========================= */

#if defined(ROBOT_SENTRY)
#include "sentry_def.h" // 仅 sentry 参数
#elif defined(ROBOT_INFANTRY)
#include "infantry_def.h" // 仅 infantry 参数
#else
#error "未定义机器人类型：请通过 make ROBOT=sentry|infantry 传入 -DROBOT_*"
#endif

#endif // !ROBOT_DEF_H
