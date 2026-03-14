// app
#include "2yaw_gimbal.h"
#include "agv_chassis.h"
#include "sentry.h"
#include "vtm_cmd.h"

// module
#include "ins_task.h"
#include "sentry_task.h"

// bsp
#include "bsp_init.h"

void SentryInit()
{
    __disable_irq();
    BSPInit();
    VTMCMDInit();
    AGVChassisInit();
    GimbalInit();
    SentryOSTaskInit();
    __enable_irq();
}

void SentryControlTask()
{
    VTMCMDTask();
    AGVChassisTask();
    GimbalTask();
}

void SentrySensorTask()
{
    // INS_Task();
}
