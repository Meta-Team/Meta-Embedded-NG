#include "auto_ammo_booster.h"
#include "sentry_def.h"
#include "auto_cmd.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include <math.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机

static Publisher_t *shoot_pub;
static AUTO_Fire_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
// static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

#define FRICTION_TARGET_SPEED 40000.0f
#define JAM_BACKOFF_ANGLE (PI / 12.0f * 180.0f / PI)

#define HEAT_PER_SHOT 10.0f
#define HEAT_COOL_RATE 30.0f
#define HEAT_LIMIT (260.0f * 0.8f)
#define HEAT_READY_SPEED_THD 35000.0f

#define JAM_CURRENT_THD 8000.0f
#define JAM_CONFIRM_MS 1000.0f
#define JAM_HANDLE_MS 5000.0f

#define SHOOT_RATE 10.0f

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
} Fire_State_e;

typedef struct
{
    Jam_State_e jam_state;
    Fire_State_e fire_state;

    loader_mode_e last_load_mode;

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

    float jam_suspect_start_ms;
    float jam_handle_start_ms;
    float jam_recover_target_angle;

    uint8_t motors_enable;
    Closeloop_Type_e loader_outer_loop;
    float loader_ref;
    float friction_ref_l;
    float friction_ref_r;
} ShootSystem_t;

static ShootSystem_t shoot_sys;

static void ShootSystemInit(void)
{
    shoot_sys.jam_state = JAM_NORMAL;
    shoot_sys.fire_state = FIRE_DISABLE;

    shoot_sys.last_load_mode = LOAD_STOP;

    shoot_sys.now_ms = 0.0f;
    shoot_sys.dt_s = 0.0f;
    shoot_sys.last_tick_ms = DWT_GetTimeline_ms();

    shoot_sys.heat_now = 0.0f;

    shoot_sys.jam_suspect_start_ms = 0.0f;
    shoot_sys.jam_handle_start_ms = 0.0f;
    shoot_sys.jam_recover_target_angle = 0.0f;

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
    uint8_t friction_ready = (shoot_sys.fire_state != FIRE_DISABLE) &&
                             (shoot_sys.friction_speed_abs_avg >= HEAT_READY_SPEED_THD);

    // 使用函数内静态量，避免你现在就改 ShootSystem_t 结构体
    static uint8_t heat_tracker_inited = 0;
    static float last_loader_angle = 0.0f;
    static float accum_forward_angle = 0.0f;

    // 单发对应的电机侧角度（单位：deg）
    const float shot_angle = ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;

    // 防止偶发跳变误计热量（可按实测调整）
    const float max_valid_delta = shot_angle * 3.0f;

    float delta_angle;
    int shot_cnt;

    if (!heat_tracker_inited)
    {
        last_loader_angle = shoot_sys.loader_angle;
        accum_forward_angle = 0.0f;
        heat_tracker_inited = 1;
    }

    delta_angle = shoot_sys.loader_angle - last_loader_angle;
    last_loader_angle = shoot_sys.loader_angle;

    if (fabsf(delta_angle) > max_valid_delta)
        delta_angle = 0.0f;

    if (friction_ready)
    {
        if (delta_angle > 0.0f)
        {
            // 正转累计
            accum_forward_angle += delta_angle;
        }
        else if (delta_angle < 0.0f)
        {
            // 倒转回退累计量，但不加热
            accum_forward_angle += delta_angle;
            if (accum_forward_angle < 0.0f)
                accum_forward_angle = 0.0f;
        }

        // 按整发计热量，支持一次循环跨多发
        shot_cnt = (int)(accum_forward_angle / shot_angle);
        if (shot_cnt > 0)
        {
            shoot_sys.heat_now += (float)shot_cnt * HEAT_PER_SHOT;
            accum_forward_angle -= (float)shot_cnt * shot_angle;
        }
    }
    else
    {
        // 非可发射阶段清掉残余，避免后续误计
        accum_forward_angle = 0.0f;
    }

    // 冷却
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
            
            if (shoot_cmd_recv.state == FIRE_ON)
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

            if (shoot_sys.friction_speed_abs_avg > HEAT_READY_SPEED_THD && shoot_cmd_recv.state == FIRE_ON)
            {
                shoot_sys.fire_state = FIRE_BURST;
            }
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
            if (shoot_cmd_recv.state == FIRE_OFF)
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
            .can_handle = &hfdcan3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.5,
                .Ki = 5,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 25000,
                .MaxOut = 25000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };
    friction_config.can_init_config.tx_id = 3; // 左摩擦轮
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hfdcan3,
            .tx_id = 8,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 7.5,
                .Ki = 10,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 25000,
                .MaxOut = 25000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M3508
    };
    loader = DJIMotorInit(&loader_config);

    // shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(AUTO_Fire_Ctrl_Cmd_s));
    ShootSystemInit();
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // ShootCaptureSignals();
    // ShootUpdateHeatFSM();
    // ShootUpdateJamFSM();
    // ShootUpdateFireFSM();
    // ShootApplyOutput();

    if (shoot_cmd_recv.state == FIRE_ON)
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
        DJIMotorSetRef(friction_l, 40000);
        DJIMotorSetRef(friction_r, 40000);
        DJIMotorSetRef(loader, 720);
    }
    else
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        DJIMotorSetRef(loader, 0);
    }


    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    // PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}