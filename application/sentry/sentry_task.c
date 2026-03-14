#include "sentry_task.h"

osThreadId controlTaskHandle;
osThreadId motorTaskHandle;
osThreadId sensorTaskHandle;
osThreadId WDTTaskHandle;

void SentryOSTaskInit(void)
{
    osThreadDef(controltask, StartControlTask, osPriorityNormal, 0, 512);
    controlTaskHandle = osThreadCreate(osThread(controltask), NULL);

    osThreadDef(motortask, StartMotorTask, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(sensortask, StartSensorTask, osPriorityNormal, 0, 256);
    sensorTaskHandle = osThreadCreate(osThread(sensortask), NULL);

    osThreadDef(wdttask, StartWDTTask, osPriorityNormal, 0, 256);
    sensorTaskHandle = osThreadCreate(osThread(wdttask), NULL);
}

__attribute__((noreturn)) void StartControlTask(void const *argument)
{
    static float control_dt;
    static float control_start;
    (void)argument;
    LOGINFO("[freeRTOS] CONTROL Task Start");
    for (;;)
    {
        control_start = DWT_GetTimeline_ms();
        SentryControlTask();
        control_dt = DWT_GetTimeline_ms() - control_start;
        if (control_dt > 1)
            LOGERROR("[freeRTOS] CONTROL Task is being DELAY! dt = [%f]", &control_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMotorTask(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    (void)argument;
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

__attribute__((noreturn)) void StartSensorTask(void const *argument)
{
    static float sensor_dt;
    static float sensor_start;
    (void)argument;
    LOGINFO("[freeRTOS] SENSOR Task Start");
    for (;;)
    {
        sensor_start = DWT_GetTimeline_ms();
        SentrySensorTask();
        sensor_dt = DWT_GetTimeline_ms() - sensor_start;
        if (sensor_dt > 1)
            LOGERROR("[freeRTOS] SENSOR Task is being DELAY! dt = [%f]", &sensor_dt);
        osDelay(1);
    }
}


__attribute__((noreturn)) void StartWDTTask(void const *argument)
{
    static float wdt_dt;
    static float wdt_start;
    (void)argument;
    LOGINFO("[freeRTOS] WDT Task Start");
    for (;;)
    {
        wdt_start = DWT_GetTimeline_ms();
        SentryWDTTask();
        wdt_dt = DWT_GetTimeline_ms() - wdt_start;
        if (wdt_dt > 1)
            LOGERROR("[freeRTOS] WDT Task is being DELAY! dt = [%f]", &wdt_dt);
        osDelay(100);
    }
}
