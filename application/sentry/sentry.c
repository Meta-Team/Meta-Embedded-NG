#include "sentry.h"
#include "bsp_init.h"
// #include "dt7_cmd.h"
#include "vtm_cmd.h"
#include "agv_chassis.h"
#include "2yaw_gimbal.h"
#include "ins_task.h"
#include "sentry_task.h"

void SentryInit()
{
    __disable_irq();
    BSPInit();
    // RobotCMDInit();
    VTMCMDInit();
    AGVChassisInit();
    // GimbalInit();
    SentryOSTaskInit();
    __enable_irq();
}

void SentryControlTask()
{
    // RobotCMDTask();
    VTMCMDTask();
    AGVChassisTask();
    // GimbalTask();
}

void SentrySensorTask()
{
    // INS_Task();
}
