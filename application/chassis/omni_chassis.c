#include "omni_chassis.h"
#include "infantry_def.h"
#include "message_center.h"
#include "dji_motor.h"

#include "general_def.h"
#include "bsp_dwt.h"

#include "arm_math.h"


/* 全向轮参数 */
#define WHEEL_SPEED_SCALE (360.0f / PERIMETER_WHEEL * REDUCTION_RATIO_WHEEL)  // 轮速转换系数:m/s -> deg/s (线速度/周长×360°×减速比)
#define OMNI_TRANS_SCALE 0.70710678f                       // √2/2,轮系与车体轴成45°时的平移投影系数
#define CHASSIS_ROTATE_ARM (CHASSIS_RX_M + CHASSIS_RY_M)   // 旋转力臂(米),wz投影到各轮驱动方向上的等效半径

static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

/* 全向轮电机实例 */
static DJIMotorInstance *wheel_motor[4];  // 轮毂电机: [0]右上 [1]右下 [2]左下 [3]左上

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float vx, vy, wz;               // 底盘速度指令
static float target_wheel_speed[4];    // 目标轮速(度/秒)


/**
 * @brief 全向轮运动学解算
 *        将底盘控制指令(vx, vy, wz)解算为各轮子的速度(逆解算)
 */
static void OmniKinematicsCalculate()
{
    vx = chassis_cmd_recv.vx;  // 前进方向速度(m/s)
    vy = chassis_cmd_recv.vy;  // 横移方向速度(m/s)
    wz = chassis_cmd_recv.wz;  // 旋转角速度(rad/s)

    // 计算每个轮子的线速度(m/s)
    // 坐标系定义如下
    // X轴: 车体前进方向
    // Y轴: 车体左移方向
    // Z轴: 垂直向上 wz逆时针旋转为正
    // 四个全向轮按X型(45°)布置,每个轮的驱动方向与车体轴成45°,
    // 旋转分量在各轮驱动方向上的投影力臂相同,均为 √2/2 × (RX + RY)

    float rot = wz * CHASSIS_ROTATE_ARM;  // 旋转引起的轮线速度分量(投影前)
    float wheel_vel[4];                   // 各轮线速度(m/s)

    wheel_vel[0] = OMNI_TRANS_SCALE * ( vx + vy + rot);  // 右上 [0]
    wheel_vel[1] = OMNI_TRANS_SCALE * ( vx - vy + rot);  // 右下 [1]
    wheel_vel[2] = OMNI_TRANS_SCALE * (-vx - vy + rot);  // 左下 [2]
    wheel_vel[3] = OMNI_TRANS_SCALE * (-vx + vy + rot);  // 左上 [3]

    for (int i = 0; i < 4; i++)
    {
        target_wheel_speed[i] = wheel_vel[i] * WHEEL_SPEED_SCALE;  // m/s -> deg/s
    }
}

/**
 * @brief 设置电机参考值
 */
static void SetMotorReference()
{
    // 设置轮毂电机速度
    for (uint8_t i = 0; i < 4; i++)
    {
        DJIMotorSetRef(wheel_motor[i], target_wheel_speed[i]);
    }
}

void OmniChassisInit()
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
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
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

    // 初始化4个轮毂电机 (hfdcan1, ID 1-4)
    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_config.can_init_config.tx_id = i + 1;
        wheel_motor[i] = DJIMotorInit(&wheel_config);
    }

    // 发布订阅初始化
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    chassis_feedback_data.real_vx = 0.0f;
    chassis_feedback_data.real_vy = 0.0f;
    chassis_feedback_data.real_wz = 0.0f;
}

void OmniChassisTask()
{
    // 获取新的控制信息
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    // 运动学解算
    OmniKinematicsCalculate();

    // 设置电机参考值
    SetMotorReference();

    // 发布反馈数据(未做正解算,real_vx/vy/wz保持为0)
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
