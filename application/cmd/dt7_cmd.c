// app
#include "sentry_def.h"
#include "dt7_cmd.h"

// module
#include "remote_control.h"
#include "message_center.h"
#include "general_def.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data;   // 从底盘应用接收的反馈信息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回



void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    // 初始化控制命令的默认值
    chassis_cmd_send.vx = 0;
    chassis_cmd_send.vy = 0;
    chassis_cmd_send.wz = 0;

    gimbal_cmd_send.yaw = YAW_CHASSIS_ALIGN_ECD;
    gimbal_cmd_send.pitch = PITCH_HORIZON_ECD;

    shoot_cmd_send.shoot_mode = SHOOT_OFF; // 初始状态关闭发射
    shoot_cmd_send.load_mode = LOAD_STOP;  // 初始状态停止拨盘
    shoot_cmd_send.shoot_rate = 1.0f;    // 初始射频1发/秒
}

static void RemoteControlSet()
{
    chassis_cmd_send.vx = 0.003f * (float)rc_data[TEMP].rc.rocker_l1;  // 左垂直摇杆
    chassis_cmd_send.vy = 0; // 暂时设置为0
    chassis_cmd_send.wz = -0.005f * (float)rc_data[TEMP].rc.rocker_l_; // 左水平摇杆
    // chassis_cmd_send.wz = -0.005f * (float)rc_data[TEMP].rc.rocker_r_; // 右水平摇杆

    gimbal_cmd_send.yaw += 0.0001f * (float)rc_data[TEMP].rc.rocker_r_;  // 右水平摇杆
    gimbal_cmd_send.pitch += 0.0001f * (float)rc_data[TEMP].rc.rocker_r1;  // 右垂直摇杆

    // 摩擦轮控制,拨轮向上打为负,向下为正
    // DT7遥控器的拨轮很容易坏,请注意
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
    } else{
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
}
void RobotCMDTask()
{
    // 从底盘应用接收反馈信息
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    
    // 计算遥控器输入的控制量
    RemoteControlSet();
    
    // 发送控制信息给底盘应用
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    // 发送控制信息给云台应用
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    // 发送控制信息给发射应用
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}