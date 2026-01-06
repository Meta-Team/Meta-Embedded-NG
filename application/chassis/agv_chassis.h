#ifndef AGV_CHASSIS_H
#define AGV_CHASSIS_H
/**
 * @brief 底盘应用初始化,请在开启rtos之前调用
 * 
 */
void AGVChassisInit(void);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void AGVChassisTask(void);

#endif // !AGV_CHASSIS_H