/**
 * @file vision_protocol_26.h
 * @brief Z_LION_AutoAim2025 串口协议(嵌入式侧)封装
 */
#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_READ_FRAME_HEAD 0xAAu
#define VISION_SEND_FRAME_SOF 0x55u
#define VISION_SEND_FRAME_CMD_ID 0x0001u
#define VISION_SEND_PAYLOAD_START 0xAAu
#define VISION_SEND_PAYLOAD_END 0xA5u

#define VISION_READ_FRAME_LEN 13u
#define VISION_SEND_FRAME_LEN 34u

typedef struct
{
    uint8_t enemy_color;
    float car_yaw;
    float car_pitch;
    uint8_t grade;
} Vision_Tx_Frame_s;

typedef struct
{
    float pitch;
    float yaw;
    float yaw_speed;
    int16_t can_shoot;
    int16_t move_state;
    int16_t if_get;
    uint8_t enemy_kind;
    int16_t enemy_x;
    int16_t enemy_y;
} Vision_Rx_Frame_s;

void VisionPackTxFrame(const Vision_Tx_Frame_s *frame, uint8_t *tx_buf, uint16_t *tx_len);

uint8_t VisionUnpackRxFrame(const uint8_t *rx_buf, uint16_t rx_len, Vision_Rx_Frame_s *frame);

#endif
