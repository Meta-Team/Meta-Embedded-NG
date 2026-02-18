#include "vtm_26.h"
#include "cmsis_os.h"
#include <string.h>
#include "bsp_log.h"
#include "crc_ref.h"

#define VTM_RX_BUFFER_SIZE 255u // 图传接收缓冲区大小
#define RC_FRAME_LEN 21u        // 遥控数据帧固定长度(字节)

static USARTInstance *vtm_usart_instance; // 图传串口实例
static vtm_info_t vtm_info;						// 图传数据

/**
 * @brief  读取图传数据,中断中读取保证速度
 * @param  buff: 读取到的图传原始数据
 * @attention  缓冲区中可能包含多帧数据(遥控帧0xA9/图传链路帧0xA5混合),
 *             使用循环逐帧解析,避免递归导致中断上下文栈溢出
 */
static void VTMReadData(uint8_t *buff)
{
    uint16_t vtm_length;       // 统计一帧数据长度
    uint16_t read_offset = 0;  // 记录相对于buff起始地址的累计偏移量,用于越界保护
    if (buff == NULL)	// 空数据包，则不作任何处理
        return;

    // 使用循环逐帧解析缓冲区中的所有数据
    // 至少需要2字节来判断帧头类型(0xA9 0x53 或 0xA5)
    while (read_offset + 2 <= VTM_RX_BUFFER_SIZE)
    {
        uint8_t *frame = buff + read_offset; // 当前帧起始地址

        // 判断帧头: 0xA9 0x53 为VTM遥控数据
        if (frame[0] == RC_SOF1 && frame[1] == RC_SOF2)
        {
            // 越界保护: 遥控帧完整长度不能超出缓冲区剩余空间
            if (read_offset + RC_FRAME_LEN > VTM_RX_BUFFER_SIZE)
                break;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(frame, RC_FRAME_LEN) == TRUE)
            {
                memcpy(&vtm_info.rc_ctrl, frame, sizeof(RC_ctrl_t));
            }
            // 遥控帧固定21字节,偏移量前移至下一帧
            read_offset += RC_FRAME_LEN;
        }
        // 判断帧头: 0xA5 为图传链路数据,按照裁判系统协议处理
        else if (frame[0] == REFEREE_SOF)
        {
            // 确保剩余空间至少能容纳一个帧头(5字节)
            if (read_offset + LEN_HEADER > VTM_RX_BUFFER_SIZE)
                break;

            memcpy(&vtm_info.FrameHeader, frame, LEN_HEADER);

            if (Verify_CRC8_Check_Sum(frame, LEN_HEADER) == TRUE)
            {
                vtm_length = (uint16_t)(vtm_info.FrameHeader.DataLength + LEN_HEADER + LEN_CMDID + LEN_TAIL);

                // 越界保护: 当前帧完整长度不能超出缓冲区剩余空间
                if (read_offset + vtm_length > VTM_RX_BUFFER_SIZE)
                    break;

                if (Verify_CRC16_Check_Sum(frame, vtm_length) == TRUE)
                {
                    vtm_info.CmdID = (frame[6] << 8 | frame[5]);
                    // TODO: 图传链路数据解析(未实现)
                }
            }

            // 按帧头中声明的长度跳过当前帧,无论校验是否通过
            uint16_t frame_len = (uint16_t)(sizeof(xFrameHeader) + LEN_CMDID + vtm_info.FrameHeader.DataLength + LEN_TAIL);
            read_offset += frame_len;
        }
        else
        {
            // 无法识别的帧头,停止解析
            break;
        }
    }
}

static void VTMRxCallback()
{
    VTMReadData(vtm_usart_instance->recv_buff);
}

static void VTMLostCallback(void *arg)
{
    USARTServiceInit(vtm_usart_instance);
    LOGWARNING("[vtm] lost vtm data");
}

vtm_info_t *VTMInit(UART_HandleTypeDef *vtm_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = VTMRxCallback;
    conf.usart_handle = vtm_usart_handle;
    conf.recv_buff_size = VTM_RX_BUFFER_SIZE; // mx 255(u8)
    vtm_usart_instance = USARTRegister(&conf);

    // Daemon_Init_Config_s daemon_conf = {
    //     .callback = VTMLostCallback,
    //     .owner_id = vtm_usart_instance,
    //     .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    // };
    // DaemonRegister(&daemon_conf);

    return &vtm_info;
}

void VTMSend(uint8_t *send, uint16_t tx_len)
{
    USARTSend(vtm_usart_instance, send, tx_len, USART_TRANSFER_DMA);
    osDelay(115);
}