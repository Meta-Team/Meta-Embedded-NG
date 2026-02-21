#include "ammo_booster.h"
#include "sentry_def.h"
#include "vtm_cmd.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include <math.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机

static Publisher_t *shoot_pub;
static Shoot_FSM_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
// static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

#define FRICTION_TARGET_SPEED 30000.0f
#define SINGLE_COMPENSATION_ANGLE 3.0f
#define JAM_BACKOFF_ANGLE (PI / 12.0f * 180.0f / PI)

#define HEAT_SPIKE_CURRENT_THD 1200.0f
#define HEAT_SPIKE_CONFIRM_MS 20.0f
#define HEAT_PER_SHOT 10.0f
#define HEAT_COOL_RATE 35.0f
#define HEAT_LIMIT 220.0f
#define HEAT_READY_SPEED_THD 4000.0f

#define JAM_CURRENT_THD 3800.0f
#define JAM_CONFIRM_MS 300.0f
#define JAM_HANDLE_MS 200.0f

#define SHOOT_RATE 1.0f

#define STOPFIRE_SPIKE_CURRENT_THD 3200.0f
#define STOPFIRE_TIMEOUT_MS 1000.0f
#define STOPFIRE_HOLD_MS 150.0f

typedef enum
{
    HEAT_STOP = 0,
    HEAT_READY,
    HEAT_SUSPECT,
    HEAT_CONFIRMED,
} Heat_Detect_State_e;

typedef enum
{
    JAM_NORMAL = 0,
    JAM_SUSPECT,
    JAM_CONFIRMED,
    JAM_HANDLE,
} Jam_State_e;

typedef enum
{
    FIRE_DISABLE = 0,
    FIRE_CEASE,
    FIRE_BURST,
    FIRE_SINGLE,
} Fire_State_e;

typedef enum
{
    STOPFIRE_DISABLE = 0,
    STOPFIRE_ACTIVE,
    STOPFIRE_PROCESS,
    STOPFIRE_HOLD,
} StopFire_State_e;

typedef struct
{
    Heat_Detect_State_e heat_state;
    Jam_State_e jam_state;
    Fire_State_e fire_state;
    StopFire_State_e stopfire_state;

    loader_mode_e last_load_mode;
    uint8_t single_queue;
    uint8_t single_just_started;

    float now_ms;
    float dt_s;
    float last_tick_ms;

    float friction_l_current;
    float friction_r_current;
    float friction_current_abs_max;
    float friction_speed_abs_avg;
    float loader_current_abs;
    float loader_angle;

    float heat_now;
    float heat_suspect_start_ms;
    uint8_t heat_confirmed_pulse;
    uint8_t heat_spike_latched;

    float jam_suspect_start_ms;
    float jam_handle_start_ms;
    float jam_recover_target_angle;

    float single_target_angle;
    float stopfire_start_ms;
    uint8_t stopfire_lock_once;
    float stopfire_lock_angle;

    uint8_t motors_enable;
    Closeloop_Type_e loader_outer_loop;
    float loader_ref;
    float friction_ref_l;
    float friction_ref_r;
} ShootSystem_t;

static ShootSystem_t shoot_sys;

static void ShootSystemInit(void)
{
    shoot_sys.heat_state = HEAT_STOP;
    shoot_sys.jam_state = JAM_NORMAL;
    shoot_sys.fire_state = FIRE_DISABLE;
    shoot_sys.stopfire_state = STOPFIRE_DISABLE;

    shoot_sys.last_load_mode = LOAD_STOP;
    shoot_sys.single_queue = 0;
    shoot_sys.single_just_started = 0;

    shoot_sys.now_ms = 0.0f;
    shoot_sys.dt_s = 0.0f;
    shoot_sys.last_tick_ms = DWT_GetTimeline_ms();

    shoot_sys.heat_now = 0.0f;
    shoot_sys.heat_suspect_start_ms = 0.0f;
    shoot_sys.heat_confirmed_pulse = 0;
    shoot_sys.heat_spike_latched = 0;

    shoot_sys.jam_suspect_start_ms = 0.0f;
    shoot_sys.jam_handle_start_ms = 0.0f;
    shoot_sys.jam_recover_target_angle = 0.0f;

    shoot_sys.single_target_angle = 0.0f;
    shoot_sys.stopfire_start_ms = 0.0f;
    shoot_sys.stopfire_lock_once = 0;
    shoot_sys.stopfire_lock_angle = 0.0f;

    shoot_sys.motors_enable = 0;
    shoot_sys.loader_outer_loop = SPEED_LOOP;
    shoot_sys.loader_ref = 0.0f;
    shoot_sys.friction_ref_l = 0.0f;
    shoot_sys.friction_ref_r = 0.0f;
}

static void ShootCaptureSignals(void)
{
    // 读取当前时间戳和电机状态信号,更新系统状态变量
    float now = DWT_GetTimeline_ms();
    float dt_ms = now - shoot_sys.last_tick_ms;
    if (dt_ms < 0.0f)
        dt_ms = 0.0f;

    shoot_sys.now_ms = now;
    shoot_sys.dt_s = dt_ms * 0.001f;
    shoot_sys.last_tick_ms = now;

    shoot_sys.friction_l_current = (float)friction_l->measure.real_current;
    shoot_sys.friction_r_current = (float)friction_r->measure.real_current;
    shoot_sys.friction_current_abs_max = fmaxf(fabsf(shoot_sys.friction_l_current), fabsf(shoot_sys.friction_r_current));
    shoot_sys.friction_speed_abs_avg = 0.5f * (fabsf(friction_l->measure.speed_aps) + fabsf(friction_r->measure.speed_aps));

    shoot_sys.loader_current_abs = fabsf((float)loader->measure.real_current);
    shoot_sys.loader_angle = loader->measure.total_angle;
}

static void ShootUpdateHeatFSM(void)
{
    // 热量检测状态机,根据摩擦轮速度和电流的变化检测是否发生了热量激增,并据此更新heat_now变量
    // friction_ready表示当前摩擦轮是否达到热量检测的条件
    uint8_t friction_ready = (shoot_sys.fire_state != FIRE_DISABLE)
                            && (shoot_sys.friction_speed_abs_avg >= HEAT_READY_SPEED_THD);
    shoot_sys.heat_confirmed_pulse = 0;

    // 如果摩擦轮不满足热量检测条件或者电流回落到阈值以下,则解除锁定
    if (!friction_ready)
        shoot_sys.heat_spike_latched = 0;
    else if (shoot_sys.friction_current_abs_max <= HEAT_SPIKE_CURRENT_THD)
        shoot_sys.heat_spike_latched = 0;

    switch (shoot_sys.heat_state)
    {
    case HEAT_STOP: // 停机
        if (friction_ready)
            shoot_sys.heat_state = HEAT_READY;
        break;

    case HEAT_READY: // 准备检测
        if (!friction_ready)
        {
            shoot_sys.heat_state = HEAT_STOP;
        }
        else if (!shoot_sys.heat_spike_latched && shoot_sys.friction_current_abs_max > HEAT_SPIKE_CURRENT_THD)
        {
            shoot_sys.heat_state = HEAT_SUSPECT;
            shoot_sys.heat_suspect_start_ms = shoot_sys.now_ms;
        }
        break;

    case HEAT_SUSPECT: // 发射嫌疑
        if (!friction_ready)
        {
            shoot_sys.heat_state = HEAT_STOP;
        }
        else if (shoot_sys.friction_current_abs_max <= HEAT_SPIKE_CURRENT_THD)
        {
            shoot_sys.heat_state = HEAT_READY;
        }
        else if ((shoot_sys.now_ms - shoot_sys.heat_suspect_start_ms) >= HEAT_SPIKE_CONFIRM_MS)
        {
            shoot_sys.heat_state = HEAT_CONFIRMED;
        }
        break;

    case HEAT_CONFIRMED: // 确认发射
        shoot_sys.heat_now += HEAT_PER_SHOT;
        shoot_sys.heat_confirmed_pulse = 1;
        shoot_sys.heat_spike_latched = 1;
        shoot_sys.heat_state = HEAT_READY;
        break;

    default:
        shoot_sys.heat_state = HEAT_STOP;
        break;
    }


    // 根据冷却速率降低当前热量heat_now
    if (shoot_sys.heat_now > 0.0f)
    {
        shoot_sys.heat_now -= HEAT_COOL_RATE * shoot_sys.dt_s;
        if (shoot_sys.heat_now < 0.0f)
            shoot_sys.heat_now = 0.0f;
    }
}

static void ShootUpdateJamFSM(void)

{
    // 首先运行最外层的卡弹有限状态机
    uint8_t jam_monitor_enable = (shoot_sys.fire_state != FIRE_DISABLE);

    if (!jam_monitor_enable)
    {
        if (shoot_sys.jam_state != JAM_HANDLE)
            shoot_sys.jam_state = JAM_NORMAL;
        else if ((shoot_sys.now_ms - shoot_sys.jam_handle_start_ms) >= JAM_HANDLE_MS)
            shoot_sys.jam_state = JAM_NORMAL;
        return;
    }

    switch (shoot_sys.jam_state)
    {
    case JAM_NORMAL: // 常规
        if (shoot_sys.loader_current_abs > JAM_CURRENT_THD)
        {
            shoot_sys.jam_state = JAM_SUSPECT;
            shoot_sys.jam_suspect_start_ms = shoot_sys.now_ms;
        }
        break;

    case JAM_SUSPECT: // 嫌疑
        if (shoot_sys.loader_current_abs <= JAM_CURRENT_THD)
        {
            shoot_sys.jam_state = JAM_NORMAL;
        }
        else if ((shoot_sys.now_ms - shoot_sys.jam_suspect_start_ms) >= JAM_CONFIRM_MS)
        {
            shoot_sys.jam_state = JAM_CONFIRMED;
        }
        break;

    case JAM_CONFIRMED: // 确认
        shoot_sys.jam_state = JAM_HANDLE;
        shoot_sys.jam_handle_start_ms = shoot_sys.now_ms;
        shoot_sys.jam_recover_target_angle = shoot_sys.loader_angle - JAM_BACKOFF_ANGLE;
        shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
        shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
        shoot_sys.loader_outer_loop = ANGLE_LOOP;
        shoot_sys.loader_ref = shoot_sys.jam_recover_target_angle;
        break;

    case JAM_HANDLE: // 处理
        if ((shoot_sys.now_ms - shoot_sys.jam_handle_start_ms) >= JAM_HANDLE_MS)
            shoot_sys.jam_state = JAM_NORMAL;
        break;

    default:
        shoot_sys.jam_state = JAM_NORMAL;
        break;
    }
}

static void ShootUpdateStopFireFSM(void)
{
    switch (shoot_sys.stopfire_state)
    {
    case STOPFIRE_DISABLE:
        if (shoot_sys.single_just_started)
        {
            shoot_sys.single_just_started = 0;

            // 设定单发参数：摩擦轮高速旋转，拨盘切角度环
            // 目标角度 = 当前角度 + (单颗子弹角度 + 防空程补偿) * 减速比
            shoot_sys.motors_enable = 1;
            shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
            shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
            shoot_sys.single_target_angle = shoot_sys.loader_angle
                + (ONE_BULLET_DELTA_ANGLE + SINGLE_COMPENSATION_ANGLE) * REDUCTION_RATIO_LOADER;
            shoot_sys.loader_outer_loop = ANGLE_LOOP;
            shoot_sys.loader_ref = shoot_sys.single_target_angle;

            // 进入激活状态，开始监视摩擦轮电流
            shoot_sys.stopfire_state = STOPFIRE_ACTIVE;
            shoot_sys.stopfire_start_ms = shoot_sys.now_ms;
            shoot_sys.stopfire_lock_once = 0;
        }
        break;

    case STOPFIRE_ACTIVE:
        // 保持摩擦轮高速旋转和拨盘角度目标
        shoot_sys.motors_enable = 1;
        shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
        shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
        shoot_sys.loader_outer_loop = ANGLE_LOOP;
        shoot_sys.loader_ref = shoot_sys.single_target_angle;

        // 检测子弹出膛：摩擦轮电流尖峰超过阈值，或等待超时防止死等
        if (shoot_sys.friction_current_abs_max > STOPFIRE_SPIKE_CURRENT_THD
            || (shoot_sys.now_ms - shoot_sys.stopfire_start_ms) >= STOPFIRE_TIMEOUT_MS)
        {
            shoot_sys.stopfire_state = STOPFIRE_PROCESS;
        }
        break;

    case STOPFIRE_PROCESS:
        // 子弹已出膛（或超时），立即锁定拨盘当前角度，精准掐断供弹防止双发
        if (!shoot_sys.stopfire_lock_once)
        {
            shoot_sys.stopfire_lock_angle = shoot_sys.loader_angle;
            shoot_sys.stopfire_lock_once = 1;
        }
        shoot_sys.motors_enable = 1;
        shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
        shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
        shoot_sys.loader_outer_loop = ANGLE_LOOP;
        shoot_sys.loader_ref = shoot_sys.stopfire_lock_angle;

        // 进入驻留状态，保持角度锁定直到拨盘完全静止
        shoot_sys.stopfire_state = STOPFIRE_HOLD;
        shoot_sys.stopfire_start_ms = shoot_sys.now_ms; // 复用计时器记录驻留起始时间
        break;

    case STOPFIRE_HOLD:
        // 驻留期间持续输出角度锁定，防止被 FIRE_CEASE 的速度环覆盖
        shoot_sys.motors_enable = 1;
        shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
        shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
        shoot_sys.loader_outer_loop = ANGLE_LOOP;
        shoot_sys.loader_ref = shoot_sys.stopfire_lock_angle;

        // 驻留超时后拨盘已静止，交出控制权给发射FSM
        if ((shoot_sys.now_ms - shoot_sys.stopfire_start_ms) >= STOPFIRE_HOLD_MS)
        {
            shoot_sys.stopfire_state = STOPFIRE_DISABLE;
            shoot_sys.fire_state = FIRE_CEASE;
        }
        break;

    default:
        shoot_sys.stopfire_state = STOPFIRE_DISABLE;
        break;
    }
}

static void ShootUpdateFireFSM(void)
{
    if (shoot_sys.jam_state == JAM_NORMAL || shoot_sys.jam_state == JAM_SUSPECT)
    {
        // 没有卡弹,继续执行内层状态机
        // 发射有限状态机

        switch (shoot_sys.fire_state)
        {
        case FIRE_DISABLE:
            shoot_sys.motors_enable = 0;
            shoot_sys.friction_ref_l = 0;
            shoot_sys.friction_ref_r = 0;
            shoot_sys.loader_outer_loop = SPEED_LOOP;
            shoot_sys.loader_ref = 0;
            
            if (shoot_cmd_recv.state == CLICK_SINGLE_PENDING || shoot_cmd_recv.state == CLICK_LONG_HOLD)
            {
                shoot_sys.fire_state = FIRE_CEASE;
            }
            break;
        
        case FIRE_CEASE:
            shoot_sys.motors_enable = 1;
            shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
            shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
            shoot_sys.loader_outer_loop = SPEED_LOOP;
            shoot_sys.loader_ref = 0;

            if (shoot_sys.friction_speed_abs_avg > HEAT_READY_SPEED_THD)
            {
                if (shoot_cmd_recv.state == CLICK_SINGLE_PENDING)
                {
                    shoot_sys.fire_state = FIRE_SINGLE;
                    shoot_sys.single_just_started = 1;
                }
                else if (shoot_cmd_recv.state == CLICK_LONG_HOLD)
                {
                    shoot_sys.fire_state = FIRE_BURST;
                }
            }
            break;
        
        case FIRE_SINGLE:
            // 激活停火有限状态机
            ShootUpdateStopFireFSM();
            break;
        
        case FIRE_BURST:
            if (shoot_sys.heat_now < HEAT_LIMIT)
            {
                shoot_sys.motors_enable = 1;
                shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
                shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
                shoot_sys.loader_outer_loop = SPEED_LOOP;
                shoot_sys.loader_ref = SHOOT_RATE * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
            }
            else
            {
                shoot_sys.motors_enable = 1;
                shoot_sys.friction_ref_l = FRICTION_TARGET_SPEED;
                shoot_sys.friction_ref_r = FRICTION_TARGET_SPEED;
                shoot_sys.loader_outer_loop = SPEED_LOOP;
                shoot_sys.loader_ref = 0;  // 停止拨盘供弹
            }
            if (shoot_cmd_recv.state == CLICK_RELEASED)
            {
                shoot_sys.fire_state = FIRE_CEASE;
            }
            break;

        default:
            break;
        }
    }
    else
    {
        // 卡弹接管期间，复位内层状态机，防止卡弹恢复后残留脏状态
        shoot_sys.fire_state = FIRE_CEASE;
        shoot_sys.stopfire_state = STOPFIRE_DISABLE;
        shoot_sys.single_just_started = 0;
    }
}


static void ShootApplyOutput(void)
{
    if (!shoot_sys.motors_enable)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    DJIMotorSetRef(friction_l, shoot_sys.friction_ref_l);
    DJIMotorSetRef(friction_r, shoot_sys.friction_ref_r);

    DJIMotorOuterLoop(loader, shoot_sys.loader_outer_loop);
    DJIMotorSetRef(loader, shoot_sys.loader_ref);
}

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };
    friction_config.can_init_config.tx_id = 1; // 左摩擦轮
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
            .tx_id = 8,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508
    };
    loader = DJIMotorInit(&loader_config);

    // shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_FSM_Ctrl_Cmd_s));
    ShootSystemInit();
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    ShootCaptureSignals();
    ShootUpdateHeatFSM();
    ShootUpdateJamFSM();
    ShootUpdateFireFSM();
    ShootApplyOutput();

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    // PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}