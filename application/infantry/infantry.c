// app
#include "omni_chassis.h"
#include "gimbal.h"
#include "infantry.h"
#include "vtm_cmd.h"
// #include "auto_cmd.h"
#include "auto_ammo_booster.h"

// module
#include "daemon.h"
#include "ins_task.h"
#include "infantry_task.h"
#include "vision_26.h"

// bsp
#include "bsp_init.h"

#include "iwdg.h"

void InfantryInit()
{
    __disable_irq();
    BSPInit();
    VTMCMDInit();
    OmniChassisInit();
    InfantryOSTaskInit();
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
    __enable_irq();
}

void InfantryControlTask()
{
    VTMCMDTask();
    OmniChassisTask();
}

void InfantrySensorTask()
{
    INS_Task();
    GimbalDataTask();
}

void InfantryCommTask()
{
    // RefereeTask();
    // VisionTask();
    // SuperCapTask();
}

void InfantryWDTTask()
{
    // DaemonTask();
    // HAL_IWDG_Refresh(&hiwdg1);
}
