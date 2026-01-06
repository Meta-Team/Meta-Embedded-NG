#include "agv_chassis.h"
#include "sentry_def.h"
#include "message_center.h"
#include "dji_motor.h"

#include "general_def.h"
#include "bsp_dwt.h"

#include "arm_math.h"


/* 舵轮参数 */
#define STEER_ANGLE_TOLERANCE 5.0f  // 舵面角度容差(度),小于此值认为舵面已对齐
#define STEER_ALIGN_SPEED_THRESHOLD 360.0f  // 舵面对齐前轮速限制(度/秒)
#define WHEEL_SPEED_SCALE (360.0f / PERIMETER_WHEEL * REDUCTION_RATIO_WHEEL)  // 轮速转换系数:m/s -> deg/s (线速度/周长×360°×减速比)

static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

/* 舵轮电机实例 */
static DJIMotorInstance *wheel_motor[4];  // 轮毂电机: [0]右上 [1]右下 [2]左下 [3]左上
static DJIMotorInstance *steer_motor[4];  // 舵面电机: [0]右上 [1]右下 [2]左下 [3]左上

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float vx, vy, wz;               // 底盘速度指令
static float vx_rf, vy_rf, vx_rb, vy_rb, vx_lb, vy_lb, vx_lf, vy_lf; // 各轮速度分量
static float wheel_speed_mps[4];       // 各轮速度值(m/s)
static float wheel_angle_rad[4];       // 各轮角度(弧度)
static float wheel_angle_deg[4];       // 各轮角度(度)
static float target_wheel_speed[4];    // 目标轮速(度/秒)
static float target_steer_angle[4];    // 目标舵面角度(度)
static uint8_t wheel_reverse_flag[4];  // 轮速反转标志

/**
 * @brief 归一化角度到(-180, 180]范围(与atan2f输出一致)
 * @param angle 输入角度(度)
 * @return 归一化后的角度(度)
 */
static inline float NormalizeAngle(float angle)
{
    // 归一化到(-180, 180]区间，与atan2f输出保持一致
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 计算两个角度之间的最短路径差值
 * @param target 目标角度(度)
 * @param current 当前角度(度)
 * @return 最短路径角度差(-180, 180]
 */
static inline float CalcShortestAngleDiff(float target, float current)
{
    float diff = target - current;
    return NormalizeAngle(diff);
}

/**
 * @brief 舵轮最短路径优化算法(方案3)
 *        利用舵面±180°等效特性,选择最优转动方向
 * @param wheel_idx 轮子索引(0-3)
 * @param target_angle 期望舵面角度(度)
 * @param target_speed 期望轮速(m/s)
 */
// 舵面偏移角度数组
static const float steer_offset[4] = {
    RF_WHEEL_POS_OFFSET_ANGLE, RB_WHEEL_POS_OFFSET_ANGLE,
    LB_WHEEL_POS_OFFSET_ANGLE, LF_WHEEL_POS_OFFSET_ANGLE
};

static void SteerOptimization(uint8_t wheel_idx, float target_angle, float target_speed)
{
    // 获取当前舵面角度(加偏移后用于计算)
    float current_angle = steer_motor[wheel_idx]->measure.total_angle + steer_offset[wheel_idx];
    current_angle = NormalizeAngle(fmodf(current_angle, 360.0f));
    
    // 归一化目标角度
    target_angle = NormalizeAngle(target_angle);
    
    // 计算两个可能的目标角度
    float target1 = target_angle;
    float target2 = NormalizeAngle(target_angle + 180.0f);
    
    // 计算到两个目标的角度差(使用arm_math的fabsf)
    float diff1 = CalcShortestAngleDiff(target1, current_angle);
    float diff2 = CalcShortestAngleDiff(target2, current_angle);
    
    float abs_diff1, abs_diff2;
    arm_abs_f32(&diff1, &abs_diff1, 1);
    arm_abs_f32(&diff2, &abs_diff2, 1);
    
    // 选择转动角度更小的方案
    if (abs_diff2 < abs_diff1)
    {
        // 选择180°反向,需要反转轮速
        target_steer_angle[wheel_idx] = current_angle + diff2 - steer_offset[wheel_idx];
        wheel_reverse_flag[wheel_idx] = 1;
    }
    else
    {
        // 选择正常方向
        target_steer_angle[wheel_idx] = current_angle + diff1 - steer_offset[wheel_idx];
        wheel_reverse_flag[wheel_idx] = 0;
    }
    
    // 转换轮速:m/s -> deg/s,并应用反转标志
    float wheel_speed_dps = target_speed * WHEEL_SPEED_SCALE;
    target_wheel_speed[wheel_idx] = wheel_reverse_flag[wheel_idx] ? -wheel_speed_dps : wheel_speed_dps;
    
    // 如果舵面未对齐,限制轮速避免打滑
    float steer_error;
    float steer_error_diff = target_steer_angle[wheel_idx] - steer_motor[wheel_idx]->measure.total_angle;
    arm_abs_f32(&steer_error_diff, &steer_error, 1);
    
    // if (steer_error > STEER_ANGLE_TOLERANCE)
    // {
    //     // 舵面未对齐,降低轮速
    //     float speed_limit = STEER_ALIGN_SPEED_THRESHOLD;
    //     if (target_wheel_speed[wheel_idx] > speed_limit)
    //         target_wheel_speed[wheel_idx] = speed_limit;
    //     else if (target_wheel_speed[wheel_idx] < -speed_limit)
    //         target_wheel_speed[wheel_idx] = -speed_limit;
    // }
}

/**
 * @brief 舵轮运动学解算
 *        将底盘控制指令(vx, vy, wz)解算为各轮子的速度和舵面角度
 */
static void AGVKinematicsCalculate()
{
    vx = chassis_cmd_recv.vx;  // 前进方向速度(m/s)
    vy = chassis_cmd_recv.vy;  // 横移方向速度(m/s)
    wz = chassis_cmd_recv.wz;  // 旋转角速度(rad/s)
    
    // 计算每个轮子的线速度分量(m/s)
    // 右上轮 [0]
    vx_rf = vx - wz * CHASSIS_RY_M;
    vy_rf = vy + wz * CHASSIS_RX_M;
    
    // 右下轮 [1]
    vx_rb = vx + wz * CHASSIS_RY_M;
    vy_rb = vy + wz * CHASSIS_RX_M;
    
    // 左下轮 [2]
    vx_lb = vx + wz * CHASSIS_RY_M;
    vy_lb = vy - wz * CHASSIS_RX_M;
    
    // 左上轮 [3]
    vx_lf = vx - wz * CHASSIS_RY_M;
    vy_lf = vy - wz * CHASSIS_RX_M;
    
    // 计算各轮的速度幅值和角度(使用arm_math库)
    
    // 右上
    arm_sqrt_f32(vx_rf * vx_rf + vy_rf * vy_rf, &wheel_speed_mps[0]);
    wheel_angle_rad[0] = atan2f(vy_rf, vx_rf);
    wheel_angle_deg[0] = wheel_angle_rad[0] * RAD_2_DEGREE;
    
    // 右下
    arm_sqrt_f32(vx_rb * vx_rb + vy_rb * vy_rb, &wheel_speed_mps[1]);
    wheel_angle_rad[1] = atan2f(vy_rb, vx_rb);
    wheel_angle_deg[1] = wheel_angle_rad[1] * RAD_2_DEGREE;
    
    // 左下
    arm_sqrt_f32(vx_lb * vx_lb + vy_lb * vy_lb, &wheel_speed_mps[2]);
    wheel_angle_rad[2] = atan2f(vy_lb, vx_lb);
    wheel_angle_deg[2] = wheel_angle_rad[2] * RAD_2_DEGREE;
    
    // 左上
    arm_sqrt_f32(vx_lf * vx_lf + vy_lf * vy_lf, &wheel_speed_mps[3]);
    wheel_angle_rad[3] = atan2f(vy_lf, vx_lf);
    wheel_angle_deg[3] = wheel_angle_rad[3] * RAD_2_DEGREE;
    
    // 对每个轮子进行舵轮优化
    for (uint8_t i = 0; i < 4; i++)
    {
        SteerOptimization(i, wheel_angle_deg[i], wheel_speed_mps[i]);
    }
}

/**
 * @brief 设置电机参考值
 */
static void SetMotorReference()
{
    // 设置舵面电机角度
    for (uint8_t i = 0; i < 4; i++)
    {
        DJIMotorSetRef(steer_motor[i], target_steer_angle[i]);
        DJIMotorSetRef(wheel_motor[i], target_wheel_speed[i]);
    }
}

void AGVChassisInit()
{
    // 轮毂电机初始化配置(速度环)
    Motor_Init_Config_s wheel_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5f,
                .Ki = 0.05f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    
    // 舵面电机初始化配置(角度环+速度环)
    Motor_Init_Config_s steer_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 0,
                .Improve = PID_Derivative_On_Measurement,
                .MaxOut = 720,
            },
            .speed_PID = {
                .Kp = 4.5f,
                .Ki = 0.2f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    
    // 初始化4个轮毂电机 (hfdcan1, ID 1-4)
    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_config.can_init_config.tx_id = i + 1;
        wheel_motor[i] = DJIMotorInit(&wheel_config);
    }
    
    // 初始化4个舵面电机 (hfdcan2, ID 1-4)
    for (uint8_t i = 0; i < 4; i++)
    {
        steer_config.can_init_config.tx_id = i + 1;
        steer_motor[i] = DJIMotorInit(&steer_config);
    }
    
    // 发布订阅初始化
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void AGVChassisTask()
{
    // 获取新的控制信息
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    
    // 运动学解算
    AGVKinematicsCalculate();
    
    // 设置电机参考值
    SetMotorReference();
    
    // 反馈数据更新(可选,根据需要实现)
    // chassis_feedback_data.real_vx = ...;
    // chassis_feedback_data.real_vy = ...;
    // chassis_feedback_data.real_wz = ...;
    
    // 发布反馈数据
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}