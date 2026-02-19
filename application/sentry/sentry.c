#include "sentry.h"
#include "bsp_init.h"
#include "dt7_cmd.h"
#include "agv_chassis.h"
#include "sentry_task.h"

void SentryInit()
{
    __disable_irq();
    BSPInit();
    RobotCMDInit();
    AGVChassisInit();
    SentryOSTaskInit();
    __enable_irq();
}

void SentryControlTask()
{
    RobotCMDTask();
    AGVChassisTask();
}

void SentryServiceTask()
{
}
