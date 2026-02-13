#ifndef VTM_26_H
#define VTM_26_H

#include "referee_protocol_26.h"
#include "referee_26.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "bsp_log.h"

#define RC_SOF1 0xA9 // VTM遥控数据起始字节 
#define RC_SOF2 0x5A

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)


/* ----------------------- Data Struct ------------------------------------- */

typedef struct
{
    /*********************************************
     * 帧头 共 2 字节
     *********************************************/
    uint8_t sof_1; //0xA9
    uint8_t sof_2; //0x53

    /*********************************************
     * 遥控器数据段，共 8 字节
     *********************************************/
    union {
        uint64_t raw;
        struct {
            uint64_t stick_RH      : 11;
            uint64_t stick_RV      : 11;
            uint64_t stick_LV      : 11;
            uint64_t stick_LH      : 11;
            uint64_t mode_switch   : 2;
            uint64_t button_pause  : 1;
            uint64_t button_left   : 1;
            uint64_t button_right  : 1;
            uint64_t dial          : 11;
            uint64_t trigger       : 1;
            uint64_t _unused       : 3;
        } bit;
    } rc;

    /*********************************************
     * 鼠标段，共 7 字节
     *********************************************/
    union {
        uint8_t raw[7];
        struct {
            int16_t mouse_x;
            int16_t mouse_y;
            int16_t mouse_z;

            uint8_t mouse_left    : 2;
            uint8_t mouse_right   : 2;
            uint8_t mouse_middle  : 2;
            uint8_t _unused       : 2;
        } bit;
    } mouse;

    /*********************************************
     * 键盘按键，共 2 字节
     *********************************************/
    union {
        uint16_t raw;
        struct {
            uint16_t w      : 1;
            uint16_t s      : 1;
            uint16_t a      : 1;
            uint16_t d      : 1;
            uint16_t shift  : 1;
            uint16_t ctrl   : 1;
            uint16_t q      : 1;
            uint16_t e      : 1;
            uint16_t r      : 1;
            uint16_t f      : 1;
            uint16_t g      : 1;
            uint16_t z      : 1;
            uint16_t x      : 1;
            uint16_t c      : 1;
            uint16_t v      : 1;
            uint16_t b      : 1;
        } bit;
    } keyboard;

    /*********************************************
     * CRC16 校验数据
     *********************************************/
    uint16_t crc16;

} RC_ctrl_t;


#pragma pack(1)

typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;

    // 所有图传链路未实现
    // 遥控数据
    RC_ctrl_t rc_ctrl;
} vtm_info_t;

#pragma pack()

vtm_info_t *VTMInit(UART_HandleTypeDef *vtm_usart_handle);

void VTMSend(uint8_t *send, uint16_t tx_len);

#endif // !VTM_26_H