#ifndef DR_NAV_H
#define DR_NAV_H

#include <stdint.h>

typedef enum
{
    DR_NAV_IDLE = 0,
    DR_NAV_LINE,
    DR_NAV_TURN,
    DR_NAV_HOLD,
    DR_NAV_FINISHED,
} DRNavState_e;

typedef struct
{
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float wz;
} DRNavPose_s;

typedef struct
{
    float x;
    float y;
    float yaw;
    float max_vel;
    float max_wz;
    float hold_time;
} DRNavWaypoint_s;

typedef struct
{
    float vx;
    float vy;
    float wz;
} DRNavCmd_s;

void DRNavInit(void);
void DRNavLoadRoute(const DRNavWaypoint_s *route, uint8_t count);
void DRNavResetPose(float x, float y, float yaw);
void DRNavStart(void);
void DRNavStop(void);
void DRNavUpdate(float dt, float body_vx, float body_vy, float body_wz, float imu_yaw);

void DRNavGetPose(DRNavPose_s *pose);
void DRNavGetCmd(DRNavCmd_s *cmd);
DRNavState_e DRNavGetState(void);
uint8_t DRNavIsFinished(void);
uint8_t DRNavGetCurrentWaypoint(void);

#endif // DR_NAV_H
