#include "agv_chassis.h"
#include "sentry_def.h"
#include "message_center.h"
#include "dji_motor.h"

#include "general_def.h"
#include "bsp_dwt.h"

#include "arm_math.h"


/* 舵轮参数 */
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
static float wheel_speed[4];           // 各轮速度值(deg/s)
static float steer_angle[4];           // 各轮角度(度)
static float target_wheel_speed[4];    // 目标轮速(度/秒)
static float target_steer_angle[4];    // 目标舵面角度(度)
static int8_t wheel_dir_flag[4] = {1, 1, 1, 1};  // 轮速方向标志

// 舵面偏移角度数组
static const float steer_offset[4] = {
    RF_WHEEL_POS_OFFSET_ANGLE, RB_WHEEL_POS_OFFSET_ANGLE,
    LB_WHEEL_POS_OFFSET_ANGLE, LF_WHEEL_POS_OFFSET_ANGLE
};


/**
 * @brief 舵轮运动学解算
 *        将底盘控制指令(vx, vy, wz)解算为各轮子的速度和舵面角度
 */
static void AGVKinematicsCalculate()
{
    vx = chassis_cmd_recv.vx;  // 前进方向速度(m/s)
    vy = chassis_cmd_recv.vy;  // 横移方向速度(m/s)
    wz = chassis_cmd_recv.wz;  // 旋转角速度(rad/s)

    for(int i = 0; i < 4; i++)
    {
        wheel_speed[i] = wheel_motor[i]->measure.speed_aps;
        steer_angle[i] = steer_motor[i]->measure.total_angle;
    }

    // 计算每个轮子的线速度分量(m/s)
    // 坐标系定义如下
    // X轴: 车体前进方向
    // Y轴: 车体左移方向
    // Z轴: 垂直向上 wz逆时针旋转为正

    // 左上轮 [3]
    vx_lf = vx - wz * CHASSIS_RY_M;
    vy_lf = vy + wz * CHASSIS_RX_M;

    // 右上轮 [0]
    vx_rf = vx + wz * CHASSIS_RY_M;
    vy_rf = vy + wz * CHASSIS_RX_M;
    
    // 左下轮 [2]
    vx_lb = vx - wz * CHASSIS_RY_M;
    vy_lb = vy - wz * CHASSIS_RX_M;

    // 右下轮 [1]
    vx_rb = vx + wz * CHASSIS_RY_M;
    vy_rb = vy - wz * CHASSIS_RX_M;

    float wheel_vx[4] = {vx_rf, vx_rb, vx_lb, vx_lf};
    float wheel_vy[4] = {vy_rf, vy_rb, vy_lb, vy_lf};

    for (int i = 0; i < 4; i++)
    {
        float vel = sqrtf(wheel_vx[i] * wheel_vx[i] + wheel_vy[i] * wheel_vy[i]);

        if (vel < 0.001f)
        {
            target_wheel_speed[i] = 0.0f;
            continue;
        }

        float target_angle = atan2f(wheel_vy[i], wheel_vx[i]) * RAD_2_DEGREE + steer_offset[i];
        float diff = target_angle - steer_angle[i];
        
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        if (diff > 90.0f)
        {
            diff -= 180.0f;
            wheel_dir_flag[i] = -1;
        }
        else if (diff < -90.0f)
        {
            diff += 180.0f;
            wheel_dir_flag[i] = -1;
        }
        else
        {
            wheel_dir_flag[i] = 1;
        }

        target_steer_angle[i] = steer_angle[i] + diff;
        target_wheel_speed[i] = vel * WHEEL_SPEED_SCALE * wheel_dir_flag[i];
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
                .Kp = 10.0f,
                .Ki = 0.2f,
                .Kd = 0.0f,
                .IntegralLimit = 0,
                .Improve = PID_Derivative_On_Measurement,
                .MaxOut = 500,
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