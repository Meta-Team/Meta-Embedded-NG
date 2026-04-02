// app
#include "2yaw_gimbal.h"
#include "agv_chassis.h"
#include "sentry.h"
// #include "vtm_cmd.h"
#include "auto_cmd.h"
#include "auto_ammo_booster.h"

// module
#include "daemon.h"
#include "ins_task.h"
#include "sentry_task.h"
#include "vision_26.h"

// bsp
#include "bsp_init.h"

#include "iwdg.h"

void SentryInit()
{
    __disable_irq();
    BSPInit();
    // VTMCMDInit();
    AUTOCMDInit();
    AGVChassisInit();
    GimbalInit();
    // ShootInit();
    SentryOSTaskInit();
    __enable_irq();
}

void SentryControlTask()
{
    // VTMCMDTask();
    AUTOCMDTask();
    AGVChassisTask();
    GimbalTask();
    // ShootTask();
}

void SentrySensorTask()
{
    INS_Task();
    GimbalDataTask();
}

void SentryCommTask()
{
    // RefereeTask();
    VisionTask();
    // SuperCapTask();
}

void SentryWDTTask()
{
    // DaemonTask();
    // HAL_IWDG_Refresh(&hiwdg1);
}
