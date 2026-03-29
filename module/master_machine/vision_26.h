/**
 * @file vision_26.h
 * @brief 视觉通信接口(2026协议)
 */

#ifndef VISION_H
#define VISION_H

#include "usart.h"
#include "vision_protocol_26.h"

#define VISION_RECV_BUFF_SIZE 64u

typedef Vision_Tx_Frame_s Vision_Tx_s;
typedef Vision_Rx_Frame_s Vision_Rx_s;

Vision_Rx_s *VisionInit(UART_HandleTypeDef *uart_handle);

void VisionSetTxData(uint8_t enemy_color, float car_yaw, float car_pitch, uint8_t grade);

void VisionSend(void);

void VisionTask(void);

Vision_Tx_s *VisionGetTxData(void);

#endif
