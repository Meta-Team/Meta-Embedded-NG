#include "omni_chassis.h"

#include "robot_def.h"
#include "message_center.h"
#include "dji_motor.h"

#include "bsp_dwt.h"
#include "general_def.h"

#include <math.h>
#include <stdint.h>

/* 全向轮参数 */
#define SQRT_ONE_HALF 0.7071067812f
#define WHEEL_SPEED_SCALE (360.0f / PERIMETER_WHEEL * REDUCTION_RATIO_WHEEL) // m/s -> 电机轴 deg/s
#define CHASSIS_FEEDBACK_RC 0.03f
#define MIN_FEEDBACK_DT 0.0001f
#define MIN_ROTATION_RADIUS 0.0001f

/**
 * @brief 电机编号与参考图中的 0~3 号轮一致
 *
 * 坐标系：X 轴向前，Y 轴向左，Z 轴向上，wz 逆时针为正。
 * 各轮正滚动方向相对 X 轴依次为 135、-135、-45、45 度。
 */
typedef enum
{
    OMNI_WHEEL_LF = 0, // 左前轮，CAN ID 1
    OMNI_WHEEL_LB,     // 左后轮，CAN ID 2
    OMNI_WHEEL_RB,     // 右后轮，CAN ID 3
    OMNI_WHEEL_RF,     // 右前轮，CAN ID 4
} Omni_Wheel_Index_e;

static Publisher_t *chassis_pub;  // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

/* 全向轮电机实例 */
static DJIMotorInstance *wheel_motor[4]; // [0]左前 [1]左后 [2]右后 [3]右前

/* 私有函数计算的中介变量，设为静态避免参数传递的开销 */
static float target_wheel_speed[4];  // 目标电机轴速度，deg/s
static float chassis_rotation_radius; // 轮组投影点到底盘中心的距离，m
static uint32_t feedback_cnt;         // 反馈解算时间戳

static float Clampf(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

/**
 * @brief 将底盘速度指令解算为四个全向轮的目标电机轴速度
 */
static void OmniKinematicsCalculate(void)
{
    const float vx = chassis_cmd_recv.vx; // 前进方向速度，m/s
    const float vy = chassis_cmd_recv.vy; // 横移方向速度，m/s
    const float rotation_speed = chassis_cmd_recv.wz * chassis_rotation_radius; // 旋转产生的轮组线速度，m/s

    /*
     * 坐标系定义如下：
     * X 轴：车体前进方向
     * Y 轴：车体左移方向
     * Z 轴：垂直向上，wz 逆时针旋转为正
     *
     * 先计算各轮沿无滑动方向的线速度，再换算为减速箱前电机轴角速度。
     * 公式对应参考图中的 Motor0~Motor3。
     */
    // 左前轮 [0]
    target_wheel_speed[OMNI_WHEEL_LF] =
        (-SQRT_ONE_HALF * vx + SQRT_ONE_HALF * vy + rotation_speed) * WHEEL_SPEED_SCALE;

    // 左后轮 [1]
    target_wheel_speed[OMNI_WHEEL_LB] =
        (-SQRT_ONE_HALF * vx - SQRT_ONE_HALF * vy + rotation_speed) * WHEEL_SPEED_SCALE;

    // 右后轮 [2]
    target_wheel_speed[OMNI_WHEEL_RB] =
        (SQRT_ONE_HALF * vx - SQRT_ONE_HALF * vy + rotation_speed) * WHEEL_SPEED_SCALE;

    // 右前轮 [3]
    target_wheel_speed[OMNI_WHEEL_RF] =
        (SQRT_ONE_HALF * vx + SQRT_ONE_HALF * vy + rotation_speed) * WHEEL_SPEED_SCALE;
}

/**
 * @brief 设置四个行走电机的速度参考值
 */
static void SetMotorReference(void)
{
    // 设置四个全向轮电机的目标速度
    for (uint8_t i = 0; i < 4; i++)
        DJIMotorSetRef(wheel_motor[i], target_wheel_speed[i]);
}

/**
 * @brief 根据四轮速度反馈反解底盘实际速度并进行一阶低通滤波
 */
static void UpdateChassisFeedback(void)
{
    float wheel_linear_speed[4];              // 各轮沿无滑动方向的线速度，m/s
    float dt = DWT_GetDeltaT(&feedback_cnt);  // 两次反馈解算的时间间隔，s
    float vx_raw;                             // 反解得到的前进速度，m/s
    float vy_raw;                             // 反解得到的横移速度，m/s
    float wz_raw = 0.0f;                      // 反解得到的旋转角速度，rad/s
    float alpha;                              // 一阶低通滤波系数

    if (dt < MIN_FEEDBACK_DT)
        dt = MIN_FEEDBACK_DT;

    for (uint8_t i = 0; i < 4; i++)
        wheel_linear_speed[i] = wheel_motor[i]->measure.speed_aps / WHEEL_SPEED_SCALE;

    // 四轮正运动学矩阵的逆解
    vx_raw = 0.5f * SQRT_ONE_HALF *
             (-wheel_linear_speed[OMNI_WHEEL_LF] - wheel_linear_speed[OMNI_WHEEL_LB] +
              wheel_linear_speed[OMNI_WHEEL_RB] + wheel_linear_speed[OMNI_WHEEL_RF]);
    vy_raw = 0.5f * SQRT_ONE_HALF *
             (wheel_linear_speed[OMNI_WHEEL_LF] - wheel_linear_speed[OMNI_WHEEL_LB] -
              wheel_linear_speed[OMNI_WHEEL_RB] + wheel_linear_speed[OMNI_WHEEL_RF]);

    if (chassis_rotation_radius > MIN_ROTATION_RADIUS)
    {
        wz_raw = (wheel_linear_speed[OMNI_WHEEL_LF] + wheel_linear_speed[OMNI_WHEEL_LB] +
                  wheel_linear_speed[OMNI_WHEEL_RB] + wheel_linear_speed[OMNI_WHEEL_RF]) /
                 (4.0f * chassis_rotation_radius);
    }

    // 对反馈速度进行一阶低通滤波，减小电机测速噪声
    alpha = Clampf(dt / (CHASSIS_FEEDBACK_RC + dt), 0.0f, 1.0f);
    chassis_feedback_data.real_vx += alpha * (vx_raw - chassis_feedback_data.real_vx);
    chassis_feedback_data.real_vy += alpha * (vy_raw - chassis_feedback_data.real_vy);
    chassis_feedback_data.real_wz += alpha * (wz_raw - chassis_feedback_data.real_wz);
}

void OmniChassisInit(void)
{
    // 全向轮电机初始化配置（速度环）
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

    // 计算轮组投影点到底盘中心的距离
    chassis_rotation_radius = sqrtf(CHASSIS_RX_M * CHASSIS_RX_M + CHASSIS_RY_M * CHASSIS_RY_M);

    // 初始化4个全向轮电机（hfdcan1，ID 1~4依次对应左前、左后、右后、右前）
    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_config.can_init_config.tx_id = i + 1U;
        wheel_motor[i] = DJIMotorInit(&wheel_config);
    }

    // 发布订阅初始化
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    // 底盘反馈数据初始化
    chassis_feedback_data.real_vx = 0.0f;
    chassis_feedback_data.real_vy = 0.0f;
    chassis_feedback_data.real_wz = 0.0f;
    feedback_cnt = DWT->CYCCNT;
}

void OmniChassisTask(void)
{
    // 获取新的控制信息
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    // 运动学解算
    OmniKinematicsCalculate();

    // 设置电机参考值
    SetMotorReference();

    // 反馈数据更新
    UpdateChassisFeedback();

    // 发布反馈数据
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
