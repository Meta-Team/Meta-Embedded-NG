// app
#include "sentry_def.h"
#include "auto_cmd.h"

// module
#include "vtm_26.h"
#include "vision_26.h"
#include "message_center.h"
#include "general_def.h"
#include "ins_task.h"
#include "dr_nav.h"
#include "super_cap.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "arm_math.h"

#define AUTO_NAV_POINT_CNT 4
#define MATCH_START_MOVE_SPEED 1.5f
#define MATCH_START_VX_TIME_S 4.0f
#define MATCH_START_STOP_TIME_S 1.0f
#define MATCH_START_VY_TIME_S 1.5f

typedef enum
{
    MATCH_START_STAGE_IDLE = 0,
    MATCH_START_STAGE_VX,
    MATCH_START_STAGE_STOP,
    MATCH_START_STAGE_VY,
    MATCH_START_STAGE_LOOP,
} MatchStartStage_e;

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
static referee_info_t *referee_data;
static SuperCapInstance *tony_supercap;

static uint32_t ctrl_cnt; // 计算VTMControlSet的时间间隔

static float yaw_gimbal, pitch_gimbal; // 需要维护状态
static attitude_t *nav_imu_data;
static DRNavCmd_s nav_cmd_out;
static uint8_t last_mode_switch;
static uint8_t last_button_left;
static uint8_t last_button_right;
static uint8_t last_game_progress;
static MatchStartStage_e match_start_stage;
static float match_start_stage_elapsed;

static const DRNavWaypoint_s auto_nav_route[AUTO_NAV_POINT_CNT] = {
    {0.80f, 0.00f, 0.0f,      0.90f, 1.80f, 0.12f},
    {0.80f, 1.00f, PI * 0.5f, 0.85f, 1.70f, 0.12f},
    {0.00f, 1.00f, PI,         0.85f, 1.70f, 0.12f},
    {0.00f, 0.00f, -PI * 0.5f,0.90f, 1.80f, 0.12f},
};

static float WrapRad(float angle)
{
    while (angle > PI)
        angle -= PI2;
    while (angle < -PI)
        angle += PI2;
    return angle;
}

static float GetNavIMUYawRad(void)
{
    if (nav_imu_data == NULL)
        return 0.0f;
    return WrapRad(nav_imu_data->Yaw * DEGREE_2_RAD);
}

void AUTOCMDInit()
{
    vtm_data = VTMInit(&huart7);
    vision_rx_data = VisionInit(NULL);
    referee_data = RefereeInit(&huart10);
    nav_imu_data = INS_Init();

    SuperCap_Init_Config_s capconfig = {
        .can_config = {
            .can_handle = &hfdcan1,
            .rx_id = 0x498,
            .tx_id = 0x486
        },
    };
    
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(AUTO_Fire_Ctrl_Cmd_s));

    DRNavInit();
    DRNavLoadRoute(auto_nav_route, AUTO_NAV_POINT_CNT);
    DRNavResetPose(0.0f, 0.0f, GetNavIMUYawRad());

    // 初始化控制命令的默认值
    chassis_cmd_send.vx = 0;
    chassis_cmd_send.vy = 0;
    chassis_cmd_send.wz = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

    // gimbal_cmd_send.yaw = YAW_CHASSIS_ALIGN_DEG;
    // gimbal_cmd_send.pitch = PITCH_HORIZON_RAD;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;

    // yaw_gimbal = YAW_CHASSIS_ALIGN_DEG;
    // pitch_gimbal = PITCH_HORIZON_RAD;
    yaw_gimbal = 0;
    pitch_gimbal = 0;

    shoot_cmd_send.state = FIRE_OFF;

    ctrl_cnt = DWT->CYCCNT;  // 用当前计数值初始化，避免首次dt过大
    last_mode_switch = 0xFF;
    last_button_left = 0;
    last_button_right = 0;
    last_game_progress = 0xFF;
    match_start_stage = MATCH_START_STAGE_IDLE;
    match_start_stage_elapsed = 0.0f;
}

static void AUTOVTMControlSet()
{
    float delta_time = DWT_GetDeltaT(&ctrl_cnt);
    uint8_t mode_switch = vtm_data->rc_ctrl.rc.bit.mode_switch;
    uint8_t trigger_pressed = vtm_data->rc_ctrl.rc.bit.trigger;

    // if (delta_time < 0.0001f)
    //     delta_time = 0.0001f;

    #ifdef DEBUG
    if (mode_switch != last_mode_switch)
    {
        if (mode_switch == 2)
        {
            DRNavResetPose(0.0f, 0.0f, GetNavIMUYawRad());
            DRNavStart();
            LOGINFO("[auto_cmd] dr nav start");
        }
        else if (last_mode_switch == 2)
        {
            DRNavStop();
            LOGINFO("[auto_cmd] dr nav stop");
        }
        last_mode_switch = mode_switch;
    }
    #endif

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

    #ifdef DEBUG
    switch (mode_switch)
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
            gimbal_cmd_send.yaw += vision_rx_data->yaw * delta_time * 2.5;
            gimbal_cmd_send.pitch += vision_rx_data->pitch * delta_time * 2.5;
        }
        else
        {
            gimbal_cmd_send.yaw = yaw_gimbal;
            gimbal_cmd_send.pitch = pitch_gimbal;
        }
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
        shoot_cmd_send.state = trigger_pressed ? FIRE_ON : FIRE_OFF;
        break;
    case 1: // 安全模式
        // 底盘
        chassis_cmd_send.vx = 0;
        chassis_cmd_send.vy = 0;
        chassis_cmd_send.wz = 0;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

        // 云台
        gimbal_cmd_send.yaw = yaw_gimbal;
        gimbal_cmd_send.pitch = pitch_gimbal;
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;

        shoot_cmd_send.state = FIRE_OFF;
        break;
    case 2: // 自动哨兵
    {
        uint8_t button_pause = vtm_data->rc_ctrl.rc.bit.button_pause;
        uint8_t button_left = vtm_data->rc_ctrl.rc.bit.button_left;
        uint8_t button_right = vtm_data->rc_ctrl.rc.bit.button_right;

        if (button_left && !last_button_left)
        {
            DRNavResetPose(0.0f, 0.0f, GetNavIMUYawRad());
            DRNavStart();
            LOGINFO("[auto_cmd] dr nav restart");
        }
        if (button_right && !last_button_right)
        {
            DRNavStop();
            LOGINFO("[auto_cmd] dr nav pause");
        }

        last_button_left = button_left;
        last_button_right = button_right;

        if (button_pause)
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
            chassis_cmd_send.wz = 0.0f;
        }
        else
        {
            DRNavUpdate(delta_time,
                        chassis_fetch_data.real_vx,
                        chassis_fetch_data.real_vy,
                        chassis_fetch_data.real_wz,
                        GetNavIMUYawRad());
            DRNavGetCmd(&nav_cmd_out);
            chassis_cmd_send.vx = nav_cmd_out.vx;
            chassis_cmd_send.vy = nav_cmd_out.vy;
            chassis_cmd_send.wz = nav_cmd_out.wz;
        }

        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

        // 自动导航下仍保留云台人工修正
        gimbal_cmd_send.yaw = yaw_gimbal;
        gimbal_cmd_send.pitch = pitch_gimbal;
        gimbal_cmd_send.gimbal_mode = GIMBAL_LOCK;

        shoot_cmd_send.state = trigger_pressed ? FIRE_ON : FIRE_OFF;
        break;
    }
    default:
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
        shoot_cmd_send.state = FIRE_OFF;
        break;
    }
    #endif // DEBUG

    #ifndef DEBUG
    switch (mode_switch)
    {
    case 2: // 底盘云台分离
        // 底盘
        chassis_cmd_send.vx = vx_chassis;
        chassis_cmd_send.vy = vy_chassis;
        chassis_cmd_send.wz = dial * -2.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

        // 云台
        gimbal_cmd_send.yaw = yaw_gimbal;
        gimbal_cmd_send.pitch = pitch_gimbal;
        gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
        shoot_cmd_send.state = trigger_pressed ? FIRE_ON : FIRE_OFF;
        break;
    
    default:
        uint8_t game_progress = referee_data->GameState.game_progress;

        if (game_progress == 4 && last_game_progress != 4)
        {
            match_start_stage = MATCH_START_STAGE_VX;
            match_start_stage_elapsed = 0.0f;
        }
        last_game_progress = game_progress;

        switch (game_progress)
        {
        case 4:
            match_start_stage_elapsed += delta_time;
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
            shoot_cmd_send.state = FIRE_OFF;

            switch (match_start_stage)
            {
            case MATCH_START_STAGE_VX:
                chassis_cmd_send.vx = MATCH_START_MOVE_SPEED;
                chassis_cmd_send.vy = 0.0f;
                chassis_cmd_send.wz = 0.0f;
                if (match_start_stage_elapsed >= MATCH_START_VX_TIME_S)
                {
                    match_start_stage = MATCH_START_STAGE_STOP;
                    match_start_stage_elapsed = 0.0f;
                }
                break;
            case MATCH_START_STAGE_STOP:
                chassis_cmd_send.vx = 0.0f;
                chassis_cmd_send.vy = 0.0f;
                chassis_cmd_send.wz = 0.0f;
                if (match_start_stage_elapsed >= MATCH_START_STOP_TIME_S)
                {
                    match_start_stage = MATCH_START_STAGE_VY;
                    match_start_stage_elapsed = 0.0f;
                }
                break;
            case MATCH_START_STAGE_VY:
                chassis_cmd_send.vx = 0.0f;
                chassis_cmd_send.vy = MATCH_START_MOVE_SPEED;
                chassis_cmd_send.wz = 0.0f;
                if (match_start_stage_elapsed >= MATCH_START_VY_TIME_S)
                {
                    match_start_stage = MATCH_START_STAGE_LOOP;
                    match_start_stage_elapsed = 0.0f;
                }
                break;
            case MATCH_START_STAGE_LOOP:
                chassis_cmd_send.vx = 0.0f;
                chassis_cmd_send.vy = 0.0f;
                chassis_cmd_send.wz = -PI;
                // TODO: 在此编写后续循环执行逻辑
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
                gimbal_cmd_send.yaw += vision_rx_data->yaw * delta_time * 2.5;
                gimbal_cmd_send.pitch += vision_rx_data->pitch * delta_time * 2.5;
                shoot_cmd_send.state = vision_rx_data->can_shoot ? FIRE_ON : FIRE_OFF;
                break;
            default:
                chassis_cmd_send.vx = 0.0f;
                chassis_cmd_send.vy = 0.0f;
                chassis_cmd_send.wz = 0.0f;
                break;
            }
            break;
        default:
            match_start_stage = MATCH_START_STAGE_IDLE;
            match_start_stage_elapsed = 0.0f;
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
            chassis_cmd_send.wz = 0.0f;
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
            shoot_cmd_send.state = FIRE_OFF;
            break;
        }
    }
    #endif // !DEBUG
}

void AUTOCMDTask()
{
    // 从底盘应用接收反馈信息
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    
    // 从云台应用接收反馈信息
    SubGetMessage(gimbal_feed_sub, (void *)&gimbal_fetch_data);

    // 计算遥控器输入的控制量
    AUTOVTMControlSet();

    Vision_Tx_s *vision_tx_data = VisionGetTxData();
    // if (referee_data->GameRobotState.robot_id == 7)
    //     vision_tx_data->enemy_color = 2;
    // else
    //     vision_tx_data->enemy_color = 1;
    // #ifdef DEBUG
    // vision_tx_data->enemy_color = 2;
    // #endif
    vision_tx_data->enemy_color = 1;
    vision_tx_data->grade = 0;

    // uint8_t supercap_data[3];
    // supercap_data[0] = (uint8_t)100;
    // supercap_data[1] = (uint8_t)referee_data->PowerHeatData.buffer_energy;
    // supercap_data[2] = (uint8_t)0;
    // SuperCapSend(tony_supercap, supercap_data);
    
    // 发送控制信息给底盘应用
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    // 发送控制信息给云台应用
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    // 发送控制信息给发射应用
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}