#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "sentry.h"
#include "motor_task.h"

#include "bsp_dwt.h"
#include "bsp_log.h"

extern osThreadId controlTaskHandle;
extern osThreadId motorTaskHandle;
extern osThreadId serviceTaskHandle;

void SentryOSTaskInit(void);

void StartControlTask(void const *argument);
void StartMotorTask(void const *argument);
void StartServiceTask(void const *argument);