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

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回



void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

static void RemoteControlSet()
{
    chassis_cmd_send.vx = 0.01f * (float)rc_data[TEMP].rc.rocker_l1;  // 左垂直摇杆
    chassis_cmd_send.vy = 0; // 暂时设置为0
    chassis_cmd_send.wz = -0.01f * (float)rc_data[TEMP].rc.rocker_l_; // 左水平摇杆
}
void RobotCMDTask()
{
    // 从底盘应用接收反馈信息
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    
    // 计算遥控器输入的控制量
    RemoteControlSet();
    
    // 发送控制信息给底盘应用
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}