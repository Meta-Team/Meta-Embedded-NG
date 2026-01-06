#ifndef DT7_CMD_H
#define DT7_CMD_H
/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务
 * 
 */
void RobotCMDTask();

#endif // !DT7_CMD_H