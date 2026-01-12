#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
// #include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

/* DM电机CAN反馈信息 */
typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
} DM_Motor_Measure_s;

/* DM电机CAN发送信息 */
typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

/* DM电机实例 */
typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    // PIDInstance current_PID;
    // PIDInstance speed_PID;
    // PIDInstance angle_PID;
    float Kp;
    float Kd;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    
    float position_ref;                     // 位置参考值 (rad)
    float velocity_ref;                     // 速度参考值 (rad/s)
    float torque_ref;                       // 力矩参考值 (Nm)
    
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    // DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
} DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
} DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置电机运控模式参考值 (同时设置位置、速度、力矩)
 * @param motor 电机实例指针
 * @param position 目标位置 (rad), 范围: -12.5 ~ 12.5
 * @param velocity 目标速度 (rad/s), 范围: -45 ~ 45
 * @param torque 前馈力矩 (Nm), 范围: -18 ~ 18
 * @note  MIT模式公式: τ = Kp*(pos_ref - pos) + Kd*(vel_ref - vel) + torque_ref
 */
void DMMotorSetRef(DMMotorInstance *motor, float position, float velocity, float torque);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
#endif // !DMMOTOR