#include "simple_gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "ins_task.h"

static attitude_t *gimbal_IMU_data;
static float GyroDeg[3];
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void SimpleGimbalInit()
{
    gimbal_IMU_data = INS_Init();
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
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
            .other_speed_feedback_ptr = &GyroDeg[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020
    };
    // PITCH, PID初值沿用yaw,需上机整定
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
            .tx_id = 2,
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
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            .other_speed_feedback_ptr = &GyroDeg[0],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            // 装机后若电机转向与IMU反馈方向相反,调整motor_reverse_flag/feedback_reverse_flag
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务 */
void SimpleGimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_IMU:
    case GIMBAL_LOCK:
        // pitch: IMU反馈闭环,限位用电机编码器角度
        // 把IMU目标误差映射到电机编码器空间做限位,再换算回IMU目标下发
        // 注意:若机械安装使编码器角度跨越0/360翻转点,需要调零或改用total_angle
        float imu_error = gimbal_cmd_recv.pitch - gimbal_IMU_data->Pitch;
        float motor_angle = pitch_motor->measure.angle_single_round;
        float motor_target = motor_angle + imu_error;

        if (motor_target > PITCH_MAX_ECD_DEG)
        {
            motor_target = PITCH_MAX_ECD_DEG;
        }
        else if (motor_target < PITCH_MIN_ECD_DEG)
        {
            motor_target = PITCH_MIN_ECD_DEG;
        }

        DJIMotorSetRef(pitch_motor, gimbal_IMU_data->Pitch + (motor_target - motor_angle));
        break;
    default:
        break;
    }

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_IMU:
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
        break;
    case GIMBAL_LOCK:
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, YAW_CHASSIS_ALIGN_DEG);
        break;
    default:
        break;
    }

    gimbal_feedback_data.yaw_deg = yaw_motor->measure.total_angle - YAW_CHASSIS_ALIGN_DEG;
    gimbal_feedback_data.yaw_speed = yaw_motor->measure.speed_aps;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

void SimpleGimbalDataTask()
{
    GyroDeg[0] = gimbal_IMU_data->Gyro[0] * RAD_2_DEGREE;
    GyroDeg[1] = gimbal_IMU_data->Gyro[1] * RAD_2_DEGREE;
    GyroDeg[2] = gimbal_IMU_data->Gyro[2] * RAD_2_DEGREE;
}
