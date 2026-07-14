// app
#include "robot_def.h"
#include "vtm_cmd.h"

// module
#include "vtm_26.h"
#include "message_center.h"
#include "general_def.h"
#include "user_lib.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "arm_math.h"

/* ===================== 可调参数（控制手感，机型无关） ===================== */

#define CLICK_LONG_PRESS_THRESHOLD_MS 150.0f // 单击/长按判定阈值
#define CHASSIS_FOLLOW_KP -0.5f              // 底盘跟随角度环比例

/* 速度档位：本工程暂无功率环，档位仅用于缩放指令幅值。
 * 文档中 Shift(加速) 与 B(爆发) 的差异体现在功率上限，此处二者取同一幅值，B 优先。*/
#define SPEED_NORMAL_V 3.0f        // m/s   常规平移上限
#define SPEED_BOOST_V 5.0f         // m/s   加速/爆发平移上限
#define SPEED_NORMAL_W (4.0f * PI) // rad/s 常规旋转角速度
#define SPEED_BOOST_W (6.0f * PI)  // rad/s 加速/爆发旋转角速度

/* 云台灵敏度 */
#define YAW_STICK_RATE 180.0f   // deg/s 右摇杆 yaw 速率
#define PITCH_STICK_RATE 90.0f  // deg/s 右摇杆 pitch 速率
#define MOUSE_YAW_SENS 0.01f    // 鼠标每帧位移 → yaw(deg)，符号/系数需上机标定
#define MOUSE_PITCH_SENS 0.01f  // 鼠标每帧位移 → pitch(deg)，符号/系数需上机标定
#define PITCH_SOFT_MIN -30.0f   // 云台 pitch 软限位(deg)，需按机型标定
#define PITCH_SOFT_MAX 30.0f    // 云台 pitch 软限位(deg)，需按机型标定
#define ONE_KEY_TURN_DEG 180.0f // G 一键掉头角度

/* 键盘位序（与 vtm_26.h keyboard union 一致） */
#define KEYBIT_W 0
#define KEYBIT_S 1
#define KEYBIT_A 2
#define KEYBIT_D 3
#define KEYBIT_SHIFT 4
#define KEYBIT_CTRL 5
#define KEYBIT_Q 6
#define KEYBIT_E 7
#define KEYBIT_R 8
#define KEYBIT_F 9
#define KEYBIT_G 10
#define KEYBIT_Z 11
#define KEYBIT_X 12
#define KEYBIT_C 13
#define KEYBIT_V 14
#define KEYBIT_B 15

/* ============================ 模块状态 ============================ */

static Publisher_t *chassis_cmd_pub;             // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;           // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;          // 发射控制消息发布者
static Shoot_FSM_Ctrl_Cmd_s shoot_cmd_send; // 传递给发射的控制信息

static vtm_info_t *vtm_data; // 遥控器数据,初始化时返回

static Click_FSM_s trigger_click_fsm; // 扳机单击 FSM
static Click_FSM_s mouse_click_fsm;   // 鼠标左键单击 FSM

static uint16_t keyboard_last; // 上一周期键盘位,用于上升沿检测
static uint16_t key_rising;    // 本周期新按下的键位(上升沿)

static uint32_t ctrl_cnt; // 计算VTMControlSet的时间间隔

static float yaw_gimbal, pitch_gimbal; // 需要维护状态

/* 键位访问:HELD为按住(瞬时),RISE为本周期上升沿 */
#define KEY_HELD(name) (vtm_data->rc_ctrl.keyboard.bit.name)
#define KEY_RISE(bit) (key_rising & (1u << (bit)))

/* 通道归一化到[-1,1](在浮点域做减法,避免任何位宽歧义) */
static inline float NormCh(uint32_t raw)
{
    return ((float)raw - (float)RC_CH_VALUE_OFFSET) / (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
}

/* ============================ 单击 FSM ============================ */

static void ClickFSMInit(Click_FSM_s *fsm)
{
    fsm->state = CLICK_RELEASED;
    fsm->press_start_ms = 0.0f;
    fsm->long_press_threshold_ms = CLICK_LONG_PRESS_THRESHOLD_MS;
}

static void ClickFSMStep(Click_FSM_s *fsm, uint8_t pressed)
{
    float now_ms = DWT_GetTimeline_ms();
    float press_duration_ms = now_ms - fsm->press_start_ms;

    switch (fsm->state)
    {
    case CLICK_RELEASED: // 放开
        if (pressed)
        {
            fsm->state = CLICK_PRESSING;
            fsm->press_start_ms = now_ms;
        }
        break;

    case CLICK_PRESSING: // 按下
        if (!pressed)    // 放开,继续判断
        {
            if (press_duration_ms < fsm->long_press_threshold_ms)
                // 小于阈值,触发单击
                fsm->state = CLICK_SINGLE_PENDING;
            else
                // 任务调度抖动时可能出现已过阈值但本周期才看到松开,补发一次长按事件
                fsm->state = CLICK_LONG_HOLD;
        }
        else if (press_duration_ms >= fsm->long_press_threshold_ms)
        {
            // 未放开,且大于阈值,触发长按
            fsm->state = CLICK_LONG_HOLD;
        }
        // 未放开,且小于阈值,无需处理
        break;

    case CLICK_SINGLE_PENDING: // 单击
        // 单击需其他逻辑读取处理
        break;

    case CLICK_LONG_HOLD: // 长按
        if (!pressed)
            fsm->state = CLICK_RELEASED;
        break;

    default:
        fsm->state = CLICK_RELEASED;
        break;
    }
}

static uint8_t ClickFSMConsumeSingle(Click_FSM_s *fsm)
{
    if (fsm->state == CLICK_SINGLE_PENDING)
    {
        fsm->state = CLICK_RELEASED;
        return 1;
    }
    return 0;
}

/* 扳机与鼠标左键两路合并成一个开火状态 */
static Click_State_e CombineFireState(void)
{
    // 任一路长按 → 连发(优先)
    if (trigger_click_fsm.state == CLICK_LONG_HOLD || mouse_click_fsm.state == CLICK_LONG_HOLD)
        return CLICK_LONG_HOLD;

    // 任一路单击挂起 → 单发(分行调用确保两路都被消费,不被短路跳过)
    uint8_t single = 0;
    single |= ClickFSMConsumeSingle(&trigger_click_fsm);
    single |= ClickFSMConsumeSingle(&mouse_click_fsm);
    if (single)
        return CLICK_SINGLE_PENDING;

    // 按下中(尚未到阈值)
    if (trigger_click_fsm.state == CLICK_PRESSING || mouse_click_fsm.state == CLICK_PRESSING)
        return CLICK_PRESSING;

    return CLICK_RELEASED;
}

/* ============================ 输入边沿 ============================ */

static void InputEdgeUpdate(void)
{
    uint16_t now = vtm_data->rc_ctrl.keyboard.raw;
    key_rising = now & (uint16_t)~keyboard_last;
    keyboard_last = now;
}

/* ============================ 底盘控制 ============================ */

static void ChassisControlSet(void)
{
    float stick_LV = NormCh(vtm_data->rc_ctrl.rc.bit.stick_LV);
    float stick_LH = NormCh(vtm_data->rc_ctrl.rc.bit.stick_LH);
    float dial = NormCh(vtm_data->rc_ctrl.rc.bit.dial);

    // 速度档位:B(爆发) 优先于 Shift(加速)
    float vmax, wmax;
    if (KEY_HELD(b))
    {
        vmax = SPEED_BOOST_V;
        wmax = SPEED_BOOST_W;
    }
    else if (KEY_HELD(shift))
    {
        vmax = SPEED_BOOST_V;
        wmax = SPEED_BOOST_W;
    }
    else
    {
        vmax = SPEED_NORMAL_V;
        wmax = SPEED_NORMAL_W;
    }

    // 云台坐标系速度:左摇杆 + WSAD 叠加(W前 S后 A左 D右;x朝前,y朝左)
    float vx_gimbal = stick_LV * vmax + (float)(KEY_HELD(w) - KEY_HELD(s)) * vmax;
    float vy_gimbal = -stick_LH * vmax + (float)(KEY_HELD(a) - KEY_HELD(d)) * vmax;
    vx_gimbal = float_constrain(vx_gimbal, -vmax, vmax);
    vy_gimbal = float_constrain(vy_gimbal, -vmax, vmax);

    // 底盘坐标系相对云台坐标系的旋转角([0,2pi]→[-pi,pi])
    float diff_angle = gimbal_fetch_data.yaw_deg * DEGREE_2_RAD;
    if (diff_angle > PI)
        diff_angle -= PI2;

    /* 旋转矩阵:云台系速度 → 底盘系速度
     * R = [cos(diff_angle) -sin(diff_angle)]
     *     [sin(diff_angle)  cos(diff_angle)]
     */
    chassis_cmd_send.vx = arm_cos_f32(diff_angle) * vx_gimbal - arm_sin_f32(diff_angle) * vy_gimbal;
    chassis_cmd_send.vy = arm_sin_f32(diff_angle) * vx_gimbal + arm_cos_f32(diff_angle) * vy_gimbal;

    // 底盘耦合模式:mode_switch 三档
    switch (vtm_data->rc_ctrl.rc.bit.mode_switch)
    {
    case 0: // 底盘云台分离
        chassis_cmd_send.wz = dial * -wmax; // 侧拨轮手动旋转,[-1,1]→[-wmax,wmax]
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        break;
    case 1: // 底盘跟随云台
        chassis_cmd_send.wz = CHASSIS_FOLLOW_KP * diff_angle;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    case 2: // 小陀螺
        chassis_cmd_send.wz = wmax;
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    default:
        break;
    }

    // 键盘小陀螺(Q/E 按住)覆盖底盘模式:Q 正转,E 反转
    if (KEY_HELD(q) || KEY_HELD(e))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        chassis_cmd_send.wz = KEY_HELD(q) ? wmax : -wmax;
    }
}

/* ============================ 云台控制 ============================ */

static void GimbalControlSet(float dt)
{
    float stick_RH = NormCh(vtm_data->rc_ctrl.rc.bit.stick_RH);
    float stick_RV = NormCh(vtm_data->rc_ctrl.rc.bit.stick_RV);

    // 右摇杆:控制增量是摇杆对时间的积分
    yaw_gimbal += stick_RH * -YAW_STICK_RATE * dt;
    pitch_gimbal += stick_RV * PITCH_STICK_RATE * dt;

    // 鼠标:增量为每帧位移(不乘 dt);符号/系数需上机标定
    yaw_gimbal += (float)vtm_data->rc_ctrl.mouse.bit.mouse_x * MOUSE_YAW_SENS;
    pitch_gimbal += (float)vtm_data->rc_ctrl.mouse.bit.mouse_y * MOUSE_PITCH_SENS;

    // G 一键掉头(上升沿):云台位置目标 +180°(云台恒为位控,等价于目标跳变)
    if (KEY_RISE(KEYBIT_G))
        yaw_gimbal += ONE_KEY_TURN_DEG;

    // pitch 软限位(鼠标可将 pitch 拉到极限,此处夹住)
    pitch_gimbal = float_constrain(pitch_gimbal, PITCH_SOFT_MIN, PITCH_SOFT_MAX);

    gimbal_cmd_send.yaw = yaw_gimbal;
    gimbal_cmd_send.pitch = pitch_gimbal;
    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;
}

/* ============================ 发射控制 ============================ */

static void ShootControlSet(void)
{
    uint8_t trigger_pressed = vtm_data->rc_ctrl.rc.bit.trigger;
    uint8_t mouse_left_pressed = (vtm_data->rc_ctrl.mouse.bit.mouse_left != 0);

    ClickFSMStep(&trigger_click_fsm, trigger_pressed);
    ClickFSMStep(&mouse_click_fsm, mouse_left_pressed);

    shoot_cmd_send.state = CombineFireState();
}

/* ======================= 整车级组合键 ======================= */

static void SystemCommandSet(void)
{
    // Ctrl+Shift+Z:整车复位
    if (KEY_HELD(ctrl) && KEY_HELD(shift) && KEY_RISE(KEYBIT_Z))
        HAL_NVIC_SystemReset();

    /* ---- 以下依赖尚未接入的子系统(功率/裁判/超电/视觉),暂留桩 ---- */
    // TODO: 鼠标右键      → 自瞄(需在 vtm_cmd 内接入视觉)
    // TODO: V             → 能量机关/目标切换(需发射阈值与跟随联动)
    // TODO: Ctrl+Shift+X  → 裁判系统信任切换
    // TODO: Ctrl+Shift+Q  → 红/蓝方切换
    // TODO: Ctrl+Shift+W  → 底盘属性(血量/功率优先)切换
    // TODO: Ctrl+Shift+E  → 发射属性(冷却/爆发优先)切换
    // TODO: Ctrl+Shift+R  → 手动升级
    // TODO: Ctrl+Shift+F  → 手动降级
    // TODO: Ctrl+Shift+C  → 超级电容信任切换
    // TODO: Ctrl+Shift+V  → 视觉信任切换
}

/* ============================ 对外接口 ============================ */

void VTMCMDInit()
{
    vtm_data = VTMInit(&huart7);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_FSM_Ctrl_Cmd_s));

    // 初始化控制命令的默认值
    chassis_cmd_send.vx = 0;
    chassis_cmd_send.vy = 0;
    chassis_cmd_send.wz = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU;

    yaw_gimbal = 0;
    pitch_gimbal = 0;

    ClickFSMInit(&trigger_click_fsm);
    ClickFSMInit(&mouse_click_fsm);
    shoot_cmd_send.state = trigger_click_fsm.state;

    keyboard_last = 0;
    key_rising = 0;

    ctrl_cnt = DWT->CYCCNT; // 用当前计数值初始化,避免首次dt过大
}

static void VTMControlSet(void)
{
    float dt = DWT_GetDeltaT(&ctrl_cnt);

    InputEdgeUpdate();  // 先算键盘上升沿,供后续按键使用
    ChassisControlSet();
    GimbalControlSet(dt);
    ShootControlSet();
    SystemCommandSet();
}

void VTMCMDTask()
{
    // 从底盘应用接收反馈信息
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    // 从云台应用接收反馈信息
    SubGetMessage(gimbal_feed_sub, (void *)&gimbal_fetch_data);

    // 计算遥控器/键鼠输入的控制量
    VTMControlSet();

    // 发送控制信息给底盘应用
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    // 发送控制信息给云台应用
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    // 发送控制信息给发射应用
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}
