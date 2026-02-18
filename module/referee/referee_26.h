#ifndef RM_REFEREE_26_H
#define RM_REFEREE_26_H

#include "usart.h"
#include "referee_protocol_26.h"
// #include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
    uint8_t Robot_Color;		// 机器人颜色
    uint16_t Robot_ID;			// 本机器人ID
    uint16_t Cilent_ID;			// 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    game_status_t GameState;							   // 0x0001
    game_result_t GameResult;						   // 0x0002
    game_robot_HP_t GameRobotHP;					   // 0x0003
    event_data_t EventData;							   // 0x0101
    referee_warning_t RefereeWarning;				   // 0x0104
    dart_info_t DartInfo;							   // 0x0105
    robot_status_t GameRobotState;				   // 0x0201
    power_heat_data_t PowerHeatData;				   // 0x0202
    robot_pos_t GameRobotPos;					   // 0x0203
    buff_t BuffMusk;							   // 0x0204
    hurt_data_t RobotHurt;							   // 0x0206
    shoot_data_t ShootData;							   // 0x0207
    projectile_allowance_t ProjectileAllowance;	 // 0x0208
    rfid_status_t RFIDStatus;						   // 0x0209
    dart_client_cmd_t DartClientCmd;				   // 0x020A
    ground_robot_position_t GroundRobotPosition; // 0x020B
    radar_mark_data_t RadarMarkData;				   // 0x020C
    sentry_info_t SentryInfo;						   // 0x020D
    radar_info_t RadarInfo;						   // 0x020E
    
    uint8_t init_flag;

} referee_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_26_H
