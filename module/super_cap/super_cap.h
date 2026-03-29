/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"

#pragma pack(1)
typedef struct
{
    uint8_t pwr_limit; // 裁判系统功率限制(20-120W)
    uint8_t buffer_energy; // 裁判系统缓冲能量(J)
    uint8_t extra_cmd; // 额外命令
} SuperCap_Tx_s;
#pragma pack()

typedef struct
{
    float cap_voltage; // 当前电容端电压(V)
    float cap_energy; // 电容剩余能量
    uint8_t fsm_state; // 状态码
} SuperCap_Rx_s;

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    SuperCap_Tx_s cap_tx; // 发送给超电
    SuperCap_Rx_s cap_rx; // 从超电接收
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 * 
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 * 
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

#endif // !SUPER_CAP_Hd
