#include "dr_nav.h"

#include <math.h>
#include <string.h>

#include "arm_math.h"
#include "general_def.h"

#define DR_NAV_POS_REACH_M 0.05f
#define DR_NAV_YAW_REACH_RAD (3.0f * DEGREE_2_RAD)
#define DR_NAV_LINE_POS_KP 1.4f
#define DR_NAV_LINE_YAW_KP 2.2f
#define DR_NAV_TURN_YAW_KP 3.2f
#define DR_NAV_DEC_LIMIT 1.2f
#define DR_NAV_DEFAULT_MAX_VEL 0.8f
#define DR_NAV_DEFAULT_MAX_WZ 1.6f

typedef struct
{
    DRNavPose_s pose;
    DRNavCmd_s cmd;

    const DRNavWaypoint_s *route;
    uint8_t route_cnt;
    uint8_t current_idx;

    DRNavState_e state;

    float hold_timer;
    float last_imu_yaw;
    uint8_t imu_ready;
} DRNavContext_s;

static DRNavContext_s nav_ctx;

static float DRNavClampf(float val, float min, float max)
{
    if (val > max)
        return max;
    if (val < min)
        return min;
    return val;
}

static float DRNavWrapRad(float angle)
{
    while (angle > PI)
        angle -= PI2;
    while (angle < -PI)
        angle += PI2;
    return angle;
}

static void DRNavZeroCmd(void)
{
    nav_ctx.cmd.vx = 0.0f;
    nav_ctx.cmd.vy = 0.0f;
    nav_ctx.cmd.wz = 0.0f;
}

static float DRNavGetTargetMaxVel(const DRNavWaypoint_s *target)
{
    if (target->max_vel > 0.01f)
        return target->max_vel;
    return DR_NAV_DEFAULT_MAX_VEL;
}

static float DRNavGetTargetMaxWz(const DRNavWaypoint_s *target)
{
    if (target->max_wz > 0.01f)
        return target->max_wz;
    return DR_NAV_DEFAULT_MAX_WZ;
}

static void DRNavAdvanceWaypoint(void)
{
    if (nav_ctx.route_cnt == 0)
    {
        nav_ctx.state = DR_NAV_IDLE;
        DRNavZeroCmd();
        return;
    }

    if ((uint8_t)(nav_ctx.current_idx + 1U) >= nav_ctx.route_cnt)
    {
        nav_ctx.current_idx = (uint8_t)(nav_ctx.route_cnt - 1U);
        nav_ctx.state = DR_NAV_FINISHED;
        DRNavZeroCmd();
        return;
    }

    nav_ctx.current_idx++;
    nav_ctx.state = DR_NAV_LINE;
}

static void DRNavUpdatePose(float dt, float body_vx, float body_vy, float body_wz, float imu_yaw)
{
    float yaw = DRNavWrapRad(imu_yaw);
    float imu_delta;
    float imu_wz;
    float cos_yaw;
    float sin_yaw;
    float vx_world;
    float vy_world;

    if (!nav_ctx.imu_ready)
    {
        nav_ctx.last_imu_yaw = yaw;
        nav_ctx.imu_ready = 1U;
    }

    imu_delta = DRNavWrapRad(yaw - nav_ctx.last_imu_yaw);
    nav_ctx.last_imu_yaw = yaw;
    imu_wz = imu_delta / dt;

    nav_ctx.pose.yaw = yaw;
    nav_ctx.pose.wz = 0.8f * imu_wz + 0.2f * body_wz;
    nav_ctx.pose.vx = body_vx;
    nav_ctx.pose.vy = body_vy;

    cos_yaw = arm_cos_f32(yaw);
    sin_yaw = arm_sin_f32(yaw);

    vx_world = body_vx * cos_yaw - body_vy * sin_yaw;
    vy_world = body_vx * sin_yaw + body_vy * cos_yaw;

    nav_ctx.pose.x += vx_world * dt;
    nav_ctx.pose.y += vy_world * dt;
}

static void DRNavLineControl(const DRNavWaypoint_s *target, float dx, float dy, float dist, float yaw_err)
{
    float heading = atan2f(dy, dx);
    float vel_cmd = DR_NAV_LINE_POS_KP * dist;
    float vel_brake = sqrtf(2.0f * DR_NAV_DEC_LIMIT * dist);
    float max_vel = DRNavGetTargetMaxVel(target);
    float max_wz = DRNavGetTargetMaxWz(target);
    float vx_world;
    float vy_world;
    float cos_yaw;
    float sin_yaw;

    if (vel_cmd > max_vel)
        vel_cmd = max_vel;
    if (vel_cmd > vel_brake)
        vel_cmd = vel_brake;
    if (vel_cmd < 0.0f)
        vel_cmd = 0.0f;

    vx_world = vel_cmd * arm_cos_f32(heading);
    vy_world = vel_cmd * arm_sin_f32(heading);

    cos_yaw = arm_cos_f32(nav_ctx.pose.yaw);
    sin_yaw = arm_sin_f32(nav_ctx.pose.yaw);

    nav_ctx.cmd.vx = cos_yaw * vx_world + sin_yaw * vy_world;
    nav_ctx.cmd.vy = -sin_yaw * vx_world + cos_yaw * vy_world;
    nav_ctx.cmd.wz = DRNavClampf(DR_NAV_LINE_YAW_KP * yaw_err, -max_wz, max_wz);
}

static void DRNavTurnControl(const DRNavWaypoint_s *target, float yaw_err)
{
    float max_wz = DRNavGetTargetMaxWz(target);

    nav_ctx.cmd.vx = 0.0f;
    nav_ctx.cmd.vy = 0.0f;
    nav_ctx.cmd.wz = DRNavClampf(DR_NAV_TURN_YAW_KP * yaw_err, -max_wz, max_wz);
}

static void DRNavRunStateMachine(float dt)
{
    const DRNavWaypoint_s *target;
    float dx;
    float dy;
    float dist;
    float yaw_err;

    if (nav_ctx.route == NULL || nav_ctx.route_cnt == 0)
    {
        nav_ctx.state = DR_NAV_IDLE;
        DRNavZeroCmd();
        return;
    }

    if (nav_ctx.state == DR_NAV_IDLE || nav_ctx.state == DR_NAV_FINISHED)
    {
        DRNavZeroCmd();
        return;
    }

    target = &nav_ctx.route[nav_ctx.current_idx];
    dx = target->x - nav_ctx.pose.x;
    dy = target->y - nav_ctx.pose.y;
    dist = sqrtf(dx * dx + dy * dy);
    yaw_err = DRNavWrapRad(target->yaw - nav_ctx.pose.yaw);

    switch (nav_ctx.state)
    {
    case DR_NAV_LINE:
        if (dist <= DR_NAV_POS_REACH_M)
        {
            DRNavZeroCmd();
            if (fabsf(yaw_err) > DR_NAV_YAW_REACH_RAD)
            {
                nav_ctx.state = DR_NAV_TURN;
            }
            else if (target->hold_time > 0.0f)
            {
                nav_ctx.hold_timer = target->hold_time;
                nav_ctx.state = DR_NAV_HOLD;
            }
            else
            {
                DRNavAdvanceWaypoint();
            }
            break;
        }

        DRNavLineControl(target, dx, dy, dist, yaw_err);
        break;

    case DR_NAV_TURN:
        DRNavTurnControl(target, yaw_err);
        if (fabsf(yaw_err) <= DR_NAV_YAW_REACH_RAD)
        {
            DRNavZeroCmd();
            if (target->hold_time > 0.0f)
            {
                nav_ctx.hold_timer = target->hold_time;
                nav_ctx.state = DR_NAV_HOLD;
            }
            else
            {
                DRNavAdvanceWaypoint();
            }
        }
        break;

    case DR_NAV_HOLD:
        DRNavZeroCmd();
        nav_ctx.hold_timer -= dt;
        if (nav_ctx.hold_timer <= 0.0f)
        {
            DRNavAdvanceWaypoint();
        }
        break;

    default:
        nav_ctx.state = DR_NAV_IDLE;
        DRNavZeroCmd();
        break;
    }
}

void DRNavInit(void)
{
    memset(&nav_ctx, 0, sizeof(nav_ctx));
    nav_ctx.state = DR_NAV_IDLE;
}

void DRNavLoadRoute(const DRNavWaypoint_s *route, uint8_t count)
{
    nav_ctx.route = route;
    nav_ctx.route_cnt = count;
    nav_ctx.current_idx = 0U;
    nav_ctx.hold_timer = 0.0f;
    nav_ctx.state = DR_NAV_IDLE;
    DRNavZeroCmd();
}

void DRNavResetPose(float x, float y, float yaw)
{
    nav_ctx.pose.x = x;
    nav_ctx.pose.y = y;
    nav_ctx.pose.yaw = DRNavWrapRad(yaw);
    nav_ctx.pose.vx = 0.0f;
    nav_ctx.pose.vy = 0.0f;
    nav_ctx.pose.wz = 0.0f;

    nav_ctx.last_imu_yaw = nav_ctx.pose.yaw;
    nav_ctx.imu_ready = 1U;
}

void DRNavStart(void)
{
    if (nav_ctx.route == NULL || nav_ctx.route_cnt == 0)
    {
        nav_ctx.state = DR_NAV_IDLE;
        DRNavZeroCmd();
        return;
    }

    nav_ctx.current_idx = 0U;
    nav_ctx.hold_timer = 0.0f;
    nav_ctx.state = DR_NAV_LINE;
    DRNavZeroCmd();
}

void DRNavStop(void)
{
    nav_ctx.state = DR_NAV_IDLE;
    DRNavZeroCmd();
}

void DRNavUpdate(float dt, float body_vx, float body_vy, float body_wz, float imu_yaw)
{
    if (dt < 0.0001f)
        dt = 0.0001f;

    DRNavUpdatePose(dt, body_vx, body_vy, body_wz, imu_yaw);
    DRNavRunStateMachine(dt);
}

void DRNavGetPose(DRNavPose_s *pose)
{
    if (pose == NULL)
        return;
    *pose = nav_ctx.pose;
}

void DRNavGetCmd(DRNavCmd_s *cmd)
{
    if (cmd == NULL)
        return;
    *cmd = nav_ctx.cmd;
}

DRNavState_e DRNavGetState(void)
{
    return nav_ctx.state;
}

uint8_t DRNavIsFinished(void)
{
    return nav_ctx.state == DR_NAV_FINISHED;
}

uint8_t DRNavGetCurrentWaypoint(void)
{
    return nav_ctx.current_idx;
}
