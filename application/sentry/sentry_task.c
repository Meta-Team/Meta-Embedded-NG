#include "sentry_task.h"

osThreadId controlTaskHandle;
osThreadId motorTaskHandle;
osThreadId serviceTaskHandle;

void SentryOSTaskInit(void)
{
    osThreadDef(controltask, StartControlTask, osPriorityNormal, 0, 512);
    controlTaskHandle = osThreadCreate(osThread(controltask), NULL);

    osThreadDef(motortask, StartMotorTask, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(servicetask, StartServiceTask, osPriorityBelowNormal, 0, 128);
    serviceTaskHandle = osThreadCreate(osThread(servicetask), NULL);
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

__attribute__((noreturn)) void StartServiceTask(void const *argument)
{
    static float service_dt;
    static float service_start;
    (void)argument;
    LOGINFO("[freeRTOS] SERVICE Task Start");
    for (;;)
    {
        service_start = DWT_GetTimeline_ms();
        SentryServiceTask();
        service_dt = DWT_GetTimeline_ms() - service_start;
        if (service_dt > 10)
            LOGERROR("[freeRTOS] SERVICE Task is being DELAY! dt = [%f]", &service_dt);
        osDelay(10);
    }
}
