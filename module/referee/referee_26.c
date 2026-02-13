/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "referee_26.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
// #include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
// static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
    uint16_t judge_length;  // 统计一帧数据长度
    uint16_t read_offset = 0; // 记录相对于buff起始地址的累计偏移量,用于越界保护
    if (buff == NULL)	   // 空数据包，则不作任何处理
        return;

    // 使用循环代替递归,逐帧解析缓冲区中的所有数据
    // 避免递归调用在多帧粘包时导致栈溢出(中断上下文栈空间有限)
    // 循环条件: 剩余空间至少能容纳一个帧头,且当前位置为有效帧起始字节
    while (read_offset + LEN_HEADER <= RE_RX_BUFFER_SIZE && buff[read_offset] == REFEREE_SOF)
    {
        uint8_t *frame = buff + read_offset; // 当前帧起始地址

        // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
        memcpy(&referee_info.FrameHeader, frame, LEN_HEADER);

        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(frame, LEN_HEADER) == TRUE)
        {
            // 统计一帧数据长度(byte),用于CRC16校验
            // 注意 DataLength 为 uint16_t，必须使用完整 16 位长度
            judge_length = (uint16_t)(referee_info.FrameHeader.DataLength + LEN_HEADER + LEN_CMDID + LEN_TAIL);

            // 越界保护: 当前帧完整长度不能超出缓冲区剩余空间
            if (read_offset + judge_length > RE_RX_BUFFER_SIZE)
                break;

            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(frame, judge_length) == TRUE)
            {
                // 2个8位拼成16位int
                referee_info.CmdID = (frame[6] << 8 | frame[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (referee_info.CmdID)
                {
                case ID_game_status: // 0x0001
                    memcpy(&referee_info.GameState, (frame + DATA_Offset), LEN_game_status);
                    break;
                case ID_game_result: // 0x0002
                    memcpy(&referee_info.GameResult, (frame + DATA_Offset), LEN_game_result);
                    break;
                case ID_game_robot_HP: // 0x0003
                    memcpy(&referee_info.GameRobotHP, (frame + DATA_Offset), LEN_game_robot_HP);
                    break;
                case ID_event_data: // 0x0101
                    memcpy(&referee_info.EventData, (frame + DATA_Offset), LEN_event_data);
                    break;
                case ID_referee_warning: // 0x0104
                    memcpy(&referee_info.RefereeWarning, (frame + DATA_Offset), LEN_referee_warning);
                    break;
                case ID_dart_info: // 0x0105
                    memcpy(&referee_info.DartInfo, (frame + DATA_Offset), LEN_dart_info);
                    break;
                case ID_robot_status: // 0x0201
                    memcpy(&referee_info.GameRobotState, (frame + DATA_Offset), LEN_robot_status);
                    break;
                case ID_power_heat_data: // 0x0202
                    memcpy(&referee_info.PowerHeatData, (frame + DATA_Offset), LEN_power_heat_data);
                    break;
                case ID_robot_pos: // 0x0203
                    memcpy(&referee_info.GameRobotPos, (frame + DATA_Offset), LEN_robot_pos);
                    break;
                case ID_buff: // 0x0204
                    memcpy(&referee_info.BuffMusk, (frame + DATA_Offset), LEN_buff);
                    break;
                case ID_hurt_data: // 0x0206
                    memcpy(&referee_info.RobotHurt, (frame + DATA_Offset), LEN_hurt_data);
                    break;
                case ID_shoot_data: // 0x0207
                    memcpy(&referee_info.ShootData, (frame + DATA_Offset), LEN_shoot_data);
                    break;
                case ID_projectile_allowance: // 0x0208
                    memcpy(&referee_info.ProjectileAllowance, (frame + DATA_Offset), LEN_projectile_allowance);
                    break;
                case ID_rfid_status: // 0x0209
                    memcpy(&referee_info.RFIDStatus, (frame + DATA_Offset), LEN_rfid_status);
                    break;
                case ID_dart_client_cmd: // 0x020A
                    memcpy(&referee_info.DartClientCmd, (frame + DATA_Offset), LEN_dart_client_cmd);
                    break;
                case ID_ground_robot_position: // 0x020B
                    memcpy(&referee_info.GroundRobotPosition, (frame + DATA_Offset), LEN_ground_robot_position);
                    break;
                case ID_radar_mark_data: // 0x020C
                    memcpy(&referee_info.RadarMarkData, (frame + DATA_Offset), LEN_radar_mark_data);
                    break;
                case ID_sentry_info: // 0x020D
                    memcpy(&referee_info.SentryInfo, (frame + DATA_Offset), LEN_sentry_info);
                    break;
                case ID_radar_info: // 0x020E
                    memcpy(&referee_info.RadarInfo, (frame + DATA_Offset), LEN_radar_info);
                    break;
                default:
                    break;
                }
            }
        }
        // 累计偏移量前移整帧长度,指向下一帧起始位置
        // 无论CRC校验是否通过,都按帧头中声明的长度跳过当前帧
        uint16_t frame_len = (uint16_t)(sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
        read_offset += frame_len;
    }
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
    // DaemonReload(referee_daemon);
    JudgeReadData(referee_usart_instance->recv_buff);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
    USARTServiceInit(referee_usart_instance);
    LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RefereeRxCallback;
    conf.usart_handle = referee_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
    referee_usart_instance = USARTRegister(&conf);

    // Daemon_Init_Config_s daemon_conf = {
    // 	.callback = RefereeLostCallback,
    // 	.owner_id = referee_usart_instance,
    // 	.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    // };
    // referee_daemon = DaemonRegister(&daemon_conf);

    return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
    USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
    osDelay(115);
}
