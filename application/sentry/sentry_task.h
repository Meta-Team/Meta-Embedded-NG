#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "sentry.h"
#include "motor_task.h"

#include "bsp_dwt.h"
#include "bsp_log.h"

osThreadId sentryTaskHandle;
osThreadId motorTaskHandle;

void StartSENTRYTASK(void const *argument);
void StartMOTORTASK(void const *argument);

/**
 * @brief 初始化哨兵任务和电机控制任务
 *
 */
void OSTaskInit()
{
    osThreadDef(sentrytask, StartSENTRYTASK, osPriorityNormal, 0, 512);
    sentryTaskHandle = osThreadCreate(osThread(sentrytask), NULL);

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);
}

__attribute__((noreturn)) void StartSENTRYTASK(void const *argument)
{
    static float sentry_dt;
    static float sentry_start;
    LOGINFO("[freeRTOS] SENTRY Task Start");
    for (;;)
    {
        sentry_start = DWT_GetTimeline_ms();
        SentryTask();
        sentry_dt = DWT_GetTimeline_ms() - sentry_start;
        if (sentry_dt > 1)
            LOGERROR("[freeRTOS] SENTRY Task is being DELAY! dt = [%f]", &sentry_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }
}