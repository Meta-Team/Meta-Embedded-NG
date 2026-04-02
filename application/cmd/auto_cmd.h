#ifndef AUTO_CMD_H
#define AUTO_CMD_H

typedef enum
{
    FIRE_OFF = 0,
    FIRE_ON = 1,
} AUTO_Fire_State_e;

typedef struct
{
    AUTO_Fire_State_e state;
} AUTO_Fire_Ctrl_Cmd_s;

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void AUTOCMDInit();

/**
 * @brief 机器人核心控制任务
 * 
 */
void AUTOCMDTask();

#endif // !AUTO_CMD_H