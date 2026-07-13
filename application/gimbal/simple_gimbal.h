#ifndef _SIMPLE_GIMBAL_H
#define _SIMPLE_GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void SimpleGimbalInit();

/**
 * @brief 云台任务
 *
 */
void SimpleGimbalTask();

/**
 * @brief 云台数据任务,在sensor task中与INS_Task一起以1kHz调用
 *
 */
void SimpleGimbalDataTask();

#endif // _SIMPLE_GIMBAL_H
