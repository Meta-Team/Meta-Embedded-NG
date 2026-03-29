/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    SuperCap_Rx_s *msg;
    rxbuff = _instance->rx_buff;
    msg = &super_cap_instance->cap_rx;
    msg->cap_voltage = (float)(((uint16_t)rxbuff[0] << 8) | (uint16_t)rxbuff[1]) / 1000.0f;
    msg->cap_energy = (float)(((uint16_t)rxbuff[2] << 8) | (uint16_t)rxbuff[3]) / 29.78863f;
    msg->fsm_state = (uint8_t)(rxbuff[4]);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 3);
    CANSetDLC(instance->can_ins, 3);
    CANTransmit(instance->can_ins, 1);
}

SuperCap_Rx_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_rx;
}