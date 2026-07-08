#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "motor_task.h"

#include "bsp_dwt.h"
#include "bsp_log.h"

extern osThreadId controlTaskHandle;
extern osThreadId motorTaskHandle;
extern osThreadId sensorTaskHandle;
extern osThreadId commTaskHandle;
extern osThreadId WDTTaskHandle;

void RobotOSTaskInit(void);

void StartControlTask(void const *argument);
void StartMotorTask(void const *argument);
void StartSensorTask(void const *argument);
void StartCommTask(void const *argument);
void StartWDTTask(void const *argument);

#endif // !ROBOT_TASK_H
