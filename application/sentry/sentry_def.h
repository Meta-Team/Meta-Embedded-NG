#ifndef SENTRY_DEF_H
#define SENTRY_DEF_H

#define ONE_BOARD

#define VISION_USE_VCP


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
#define RF_WHEEL_POS_OFFSET_ANGLE 0.0f   // 右前轮舵面初始位置偏移角度(度)
#define RB_WHEEL_POS_OFFSET_ANGLE 0.0f  // 右后轮
#define LB_WHEEL_POS_OFFSET_ANGLE 0.0f // 左后轮
#define LF_WHEEL_POS_OFFSET_ANGLE 0.0f  // 左前轮

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

#pragma pack()
#endif // !SENTRY_DEF_H
