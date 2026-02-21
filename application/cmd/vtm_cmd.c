// app
#include "sentry_def.h"
#include "vtm_cmd.h"

// module
#include "vtm_26.h"
#include "message_center.h"
#include "general_def.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#define CLICK_LONG_PRESS_THRESHOLD_MS 150.0f

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data;   // 从底盘应用接收的反馈信息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Shoot_FSM_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息

static vtm_info_t *vtm_data; // 遥控器数据,初始化时返回

static Click_FSM_s trigger_click_fsm;

static void ClickFSMInit(void)
{
    trigger_click_fsm.state = CLICK_RELEASED;
    trigger_click_fsm.press_start_ms = 0.0f;
    trigger_click_fsm.long_press_threshold_ms = CLICK_LONG_PRESS_THRESHOLD_MS;
}

static void ClickFSMStep(uint8_t trigger_pressed)
{
    float now_ms = DWT_GetTimeline_ms();
    float press_duration_ms = now_ms - trigger_click_fsm.press_start_ms;

    switch (trigger_click_fsm.state)
    {
    case CLICK_RELEASED: // 放开
        if (trigger_pressed)
        {
            trigger_click_fsm.state = CLICK_PRESSING;
            trigger_click_fsm.press_start_ms = now_ms;
        }
        break;

    case CLICK_PRESSING: // 按下
        if (!trigger_pressed) // 放开,继续判断
        {
            if (press_duration_ms < trigger_click_fsm.long_press_threshold_ms)
                // 小于阈值，触发单击
                trigger_click_fsm.state = CLICK_SINGLE_PENDING;
            else
                // 任务调度抖动时可能出现已过阈值但本周期才看到松开,补发一次长按事件
                trigger_click_fsm.state = CLICK_LONG_HOLD;
        }
        else if (press_duration_ms >= trigger_click_fsm.long_press_threshold_ms)
        {
            // 未放开,且大于阈值，触发长按
            trigger_click_fsm.state = CLICK_LONG_HOLD;
        }
        // 未放开,且小于阈值，无需处理
        break;

    case CLICK_SINGLE_PENDING: // 单击
        // 单击需其他逻辑读取处理
        break;

    case CLICK_LONG_HOLD: // 长按
        if (!trigger_pressed)
            trigger_click_fsm.state = CLICK_RELEASED;
        break;

    default:
        trigger_click_fsm.state = CLICK_RELEASED;
        break;
    }
}

static uint8_t ClickFSMConsumeSingle(void)
{
    if (trigger_click_fsm.state == CLICK_SINGLE_PENDING)
    {
        trigger_click_fsm.state = CLICK_RELEASED;
        return 1;
    }
    return 0;
}

void VTMCMDInit()
{
    vtm_data = VTMInit(&huart7);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_FSM_Ctrl_Cmd_s));

    // 初始化控制命令的默认值
    chassis_cmd_send.vx = 0;
    chassis_cmd_send.vy = 0;
    chassis_cmd_send.wz = 0;

    gimbal_cmd_send.yaw = YAW_CHASSIS_ALIGN_ECD;
    gimbal_cmd_send.pitch = PITCH_HORIZON_ECD;

    ClickFSMInit();
    shoot_cmd_send.state = trigger_click_fsm.state;
}

static void VTMControlSet()
{
    uint8_t trigger_pressed = vtm_data->rc_ctrl.rc.bit.trigger;

    ClickFSMStep(trigger_pressed);
    shoot_cmd_send.state = trigger_click_fsm.state;
    ClickFSMConsumeSingle();
}

void VTMCMDTask()
{
    // 从底盘应用接收反馈信息
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    
    // 计算遥控器输入的控制量
    VTMControlSet();
    
    // 发送控制信息给底盘应用
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    // 发送控制信息给云台应用
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    // 发送控制信息给发射应用
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}