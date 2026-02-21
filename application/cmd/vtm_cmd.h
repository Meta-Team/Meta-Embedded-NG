#ifndef VTM_CMD_H
#define VTM_CMD_H

typedef enum
{
    CLICK_PRESSING = 0,
    CLICK_RELEASED,
    CLICK_SINGLE_PENDING,
    CLICK_LONG_HOLD,
} Click_State_e;

typedef struct
{
    Click_State_e state;
    float press_start_ms;
    float long_press_threshold_ms;
} Click_FSM_s;

typedef struct
{
    Click_State_e state;
} Shoot_FSM_Ctrl_Cmd_s;


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void VTMCMDInit();

/**
 * @brief 机器人核心控制任务
 * 
 */
void VTMCMDTask();

#endif // !VTM_CMD_H