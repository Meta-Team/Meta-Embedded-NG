// app
#include "2yaw_gimbal.h"
#include "agv_chassis.h"
#include "robot.h"
// #include "vtm_cmd.h"
#include "auto_cmd.h"
#include "auto_ammo_booster.h"

// module
#include "daemon.h"
#include "ins_task.h"
#include "robot_task.h"
#include "vision_26.h"

// bsp
#include "bsp_init.h"

#include "iwdg.h"

void RobotInit()
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    BSPInit();
    // VTMCMDInit();
    AUTOCMDInit();
    AGVChassisInit();
    GimbalInit();
    ShootInit();
    RobotOSTaskInit();
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
    // __enable_irq();
    __set_PRIMASK(primask);
}

void RobotControlTask()
{
    // VTMCMDTask();
    AUTOCMDTask();
    AGVChassisTask();
    GimbalTask();
    ShootTask();
}

void RobotSensorTask()
{
    INS_Task();
    GimbalDataTask();
}

void RobotCommTask()
{
    // RefereeTask();
    VisionTask();
    // SuperCapTask();
}

void RobotWDTTask()
{
    DaemonTask();
    HAL_IWDG_Refresh(&hiwdg1);
}
