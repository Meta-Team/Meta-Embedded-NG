#ifndef SENTRY_DEF_H
#define SENTRY_DEF_H

#include <stdint.h>

#define ONE_BOARD

#define VISION_USE_VCP

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 0     // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 0         // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 1.0f  // 3508拨盘电机的减速比,未装减速箱
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量

/* 机器人底盘的参数 */
#define CHASSIS_RY 326.0f / 2.0f     // 纵向轮距(前进后退方向),单位为mm(毫米)
#define CHASSIS_RX 326.0f / 2.0f     // 横向轮距(左右平移方向),单位为mm(毫米)
#define CHASSIS_RX_M (CHASSIS_RX / 1000.0f)  // 转换为米
#define CHASSIS_RY_M (CHASSIS_RY / 1000.0f)  // 转换为米
#define CENTER_GIMBAL_OFFSET_X 0
#define CENTER_GIMBAL_OFFSET_Y 0
#define RADIUS_WHEEL 58.0f           // 轮子半径,单位为mm(毫米)
#define RADIUS_WHEEL_M (RADIUS_WHEEL / 1000.0f)  // 转换为米
#define PERIMETER_WHEEL (RADIUS_WHEEL_M * 2 * PI) // 轮子周长
#define REDUCTION_RATIO_WHEEL 19.0f  // 电机减速比
#define RF_WHEEL_POS_OFFSET_ANGLE 160.0f   // 右前轮舵面初始位置偏移角度(度)
#define RB_WHEEL_POS_OFFSET_ANGLE -50.0f  // 右后轮
#define LB_WHEEL_POS_OFFSET_ANGLE 20.0f // 左后轮
#define LF_WHEEL_POS_OFFSET_ANGLE -150.0f  // 左前轮

#pragma pack(1)
// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} Chassis_Mode_e;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度 (m/s)
    float vy;           // 横移方向速度 (m/s)
    float wz;           // 旋转速度 (rad/s)
    float offset_angle; // 底盘和归中位置的夹角
    Chassis_Mode_e chassis_mode;
    int chassis_speed_buff;
} Chassis_Ctrl_Cmd_s;

typedef struct
{
    float real_vx;
    float real_vy;
    float real_wz;

} Chassis_Upload_Data_s;

typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
} Gimbal_Ctrl_Cmd_s;

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
#endif // !SENTRY_DEF_H
