#include "gimbal.h"
#include "sentry_def.h"
#include "dji_motor.h"
#include "xm_motor.h"
// #include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
// #include "bmi088.h"
#include "ins_task.h"
#include "bsp_dwt.h"

static attitude_t *gimbal_IMU_data;
static float GyroDeg[3];
static DJIMotorInstance *yaw_motor, *pitch_motor;
static XMMotorInstance *xm_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{
    gimbal_IMU_data = INS_Init();
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30,
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000,
                .MaxOut = 1000,
            },
            .speed_PID = {
                .Kp = 50,
                .Ki = 200,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
            .other_speed_feedback_ptr = &GyroDeg[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            // .angle_feedback_source = MOTOR_FEED,
            // .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020
    };
    // PITCH
    Motor_Init_Config_s xm_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .rx_id = 1,  // 电机ID (1~127)
        },
        .controller_param_init_config = {
            .angle_PID = { .Kp = 20.0f },   // MIT模式位置Kp
            .speed_PID = { .Kd = 1.0f },   // MIT模式阻尼Kd
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    // DWT_Delay(1); // 小米电机初始化需要1s时间
    xm_motor = XMMotorInit(&xm_config);
    XMMotorControlInit();

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    switch (gimbal_cmd_recv.chassis_mode)
    {
    case 0:
    case 1:
    case 2:
        float target_imu_angle = gimbal_cmd_recv.pitch;
        float imu_error = target_imu_angle - gimbal_IMU_data->Pitch;
        float mit_target_angle = xm_motor->measure.position  + 1.0 * imu_error * DEGREE_2_RAD;
        float mit_target_speed = xm_motor->measure.velocity - 1.0 * gimbal_IMU_data->Gyro[0];

        if (mit_target_angle > PITCH_MAX_RAD)
        {
            mit_target_angle = PITCH_MAX_RAD;
        }
        else if (mit_target_angle < PITCH_MIN_RAD)
        {
            mit_target_angle = PITCH_MIN_RAD;
        }

        if (mit_target_speed > PITCH_MAX_VEL)
        {
            mit_target_speed = PITCH_MAX_VEL;
        }
        else if (mit_target_speed < PITCH_MIN_VEL)
        {
            mit_target_speed = PITCH_MIN_VEL;
        }

        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        XMMotorEnable(xm_motor);
        XMMotorSetRef(xm_motor, mit_target_angle, mit_target_speed, 0);
        // XMMotorSetRef(xm_motor, -1, 0, 0);
        break;
    default:
        break;
    }

    gimbal_feedback_data.yaw_deg = yaw_motor->measure.total_angle - YAW_CHASSIS_ALIGN_DEG;
    gimbal_feedback_data.yaw_speed = yaw_motor->measure.speed_aps;

    // DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
    // XMMotorSetRef(xm_motor, gimbal_cmd_recv.pitch, 0.0f, 0.0f);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    // switch (gimbal_cmd_recv.gimbal_mode)
    // {
    // // 停止
    // case GIMBAL_ZERO_FORCE:
    //     DJIMotorStop(yaw_motor);
    //     DJIMotorStop(pitch_motor);
    //     break;
    // // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    // case GIMBAL_GYRO_MODE: // 后续只保留此模式
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     break;
    // // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    // case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     break;
    // default:
    //     break;
    // }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    // gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

void GimbalDataTask()
{
    GyroDeg[0] = gimbal_IMU_data->Gyro[0] * RAD_2_DEGREE;
    GyroDeg[1] = gimbal_IMU_data->Gyro[1] * RAD_2_DEGREE;
    GyroDeg[2] = gimbal_IMU_data->Gyro[2] * RAD_2_DEGREE;
}