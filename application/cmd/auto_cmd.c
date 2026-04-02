// app
#include "sentry_def.h"
#include "auto_cmd.h"

// module
#include "vtm_26.h"
#include "vision_26.h"
#include "message_center.h"
#include "general_def.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "arm_math.h"

#define CHASSIS_FOLLOW_KP -0.5f

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data;   // 从底盘应用接收的反馈信息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static AUTO_Fire_Ctrl_Cmd_s shoot_cmd_send;  // 传递给发射的控制信息

static vtm_info_t *vtm_data; // 遥控器数据,初始化时返回
static Vision_Rx_s *vision_rx_data;

static uint32_t ctrl_cnt; // 计算VTMControlSet的时间间隔

static float yaw_gimbal, pitch_gimbal; // 需要维护状态

void AUTOCMDInit()
{
    vtm_data = VTMInit(&huart7);
    vision_rx_data = VisionInit(NULL);
    Vision_Tx_s *vision_tx_data = VisionGetTxData();
    vision_tx_data->enemy_color = 2;
    vision_tx_data->grade = 0;
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(AUTO_Fire_Ctrl_Cmd_s));

    // 初始化控制命令的默认值
    chassis_cmd_send.vx = 0;
    chassis_cmd_send.vy = 0;
    chassis_cmd_send.wz = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

    // gimbal_cmd_send.yaw = YAW_CHASSIS_ALIGN_DEG;
    // gimbal_cmd_send.pitch = PITCH_HORIZON_RAD;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

    // yaw_gimbal = YAW_CHASSIS_ALIGN_DEG;
    // pitch_gimbal = PITCH_HORIZON_RAD;
    yaw_gimbal = 0;
    pitch_gimbal = 0;

    shoot_cmd_send.state = FIRE_OFF;

    ctrl_cnt = DWT->CYCCNT;  // 用当前计数值初始化，避免首次dt过大
}

static void AUTOVTMControlSet()
{
    float delta_time = DWT_GetDeltaT(&ctrl_cnt);
    // 处理摇杆
    float stick_RH, stick_RV, stick_LV, stick_LH, dial; // 归一到[-1,1]的摇杆数据
    stick_RH = (float) (vtm_data->rc_ctrl.rc.bit.stick_RH - RC_CH_VALUE_OFFSET) / (float) (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    stick_RV = (float) (vtm_data->rc_ctrl.rc.bit.stick_RV - RC_CH_VALUE_OFFSET) / (float) (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    stick_LH = (float) (vtm_data->rc_ctrl.rc.bit.stick_LH - RC_CH_VALUE_OFFSET) / (float) (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    stick_LV = (float) (vtm_data->rc_ctrl.rc.bit.stick_LV - RC_CH_VALUE_OFFSET) / (float) (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    dial = (float) (vtm_data->rc_ctrl.rc.bit.dial - RC_CH_VALUE_OFFSET) / (float) (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);

    float vx_gimbal, vy_gimbal; // 云台坐标系下的速度,x轴正方向朝前,y轴正方向朝左
    float vx_chassis, vy_chassis; // 底盘坐标系下的速度,x轴正方向朝前,y轴正方向朝左
    float diff_angle; // 底盘坐标系相对于云台坐标系的旋转角

    // 将yaw_deg([0,360]°)转换为弧度([-pi,pi])
    diff_angle = gimbal_fetch_data.yaw_deg * DEGREE_2_RAD; // [0, 2pi]
    if (diff_angle > PI)
        diff_angle -= PI2; // [-pi, pi]

    // 摇杆输入为云台坐标系
    vx_gimbal = stick_LV * 2.0f;
    vy_gimbal = stick_LH * -2.0f;

    /* 使用旋转矩阵计算底盘坐标系下的速度
     * R = [cos(diff_angle) -sin(diff_angle)]
     *     [sin(diff_angle)  cos(diff_angle)]
     */
    vx_chassis = arm_cos_f32(diff_angle) * vx_gimbal - arm_sin_f32(diff_angle) * vy_gimbal;
    vy_chassis = arm_sin_f32(diff_angle) * vx_gimbal + arm_cos_f32(diff_angle) * vy_gimbal;


    // 云台控制增量是摇杆对时间的积分
    yaw_gimbal += stick_RH * -180 * delta_time;
    pitch_gimbal += stick_RV * 90 * delta_time;

    switch (vtm_data->rc_ctrl.rc.bit.mode_switch)
    {
    case 0: // 底盘云台分离
        // 底盘
        chassis_cmd_send.vx = vx_chassis;
        chassis_cmd_send.vy = vy_chassis;
        chassis_cmd_send.wz = dial * -2.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

        // 云台
        if (vtm_data->rc_ctrl.rc.bit.button_pause)
        {
            gimbal_cmd_send.yaw += vision_rx_data->yaw * delta_time * 2;
            gimbal_cmd_send.pitch += vision_rx_data->pitch * delta_time * 2;
        }
        else
        {
            gimbal_cmd_send.yaw = yaw_gimbal;
            gimbal_cmd_send.pitch = pitch_gimbal;
        }
        gimbal_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        break;
    case 1: // 底盘跟随云台
        // 底盘
        chassis_cmd_send.vx = vx_chassis;
        chassis_cmd_send.vy = vy_chassis;
        chassis_cmd_send.wz = CHASSIS_FOLLOW_KP * diff_angle;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

        // 云台
        gimbal_cmd_send.yaw = yaw_gimbal;
        gimbal_cmd_send.pitch = pitch_gimbal;
        gimbal_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    case 2: // 小陀螺
        // 底盘
        chassis_cmd_send.vx = vx_chassis;
        chassis_cmd_send.vy = vy_chassis;
        chassis_cmd_send.wz = PI2;
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;

        // 云台
        gimbal_cmd_send.yaw = yaw_gimbal;
        gimbal_cmd_send.pitch = pitch_gimbal;
        gimbal_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    default:
        break;
    }

    // 处理扳机
    uint8_t trigger_pressed = vtm_data->rc_ctrl.rc.bit.trigger;
    shoot_cmd_send.state = trigger_pressed ? FIRE_ON : FIRE_OFF;
}

void AUTOCMDTask()
{
    // 从底盘应用接收反馈信息
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    
    // 从云台应用接收反馈信息
    SubGetMessage(gimbal_feed_sub, (void *)&gimbal_fetch_data);

    // 计算遥控器输入的控制量
    AUTOVTMControlSet();
    
    // 发送控制信息给底盘应用
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    // 发送控制信息给云台应用
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    // 发送控制信息给发射应用
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}