/**
 * @file vision_26.c
 * @brief 视觉通信接口(2026协议)
 */

#include "vision_26.h"
#include "daemon.h"
#include "bsp_log.h"
#include "sentry_def.h"
#include "ins_task.h"
#include "general_def.h"

static Vision_Rx_s vision_rx_data;
static Vision_Tx_s vision_tx_data;
static DaemonInstance *vision_daemon;

static void VisionOfflineCallback(void *owner_id)
{
#ifdef VISION_USE_UART
	USARTServiceInit((USARTInstance *)owner_id);
#endif
	LOGWARNING("[vision] communication offline, restart service.");
}

void VisionSetTxData(uint8_t enemy_color, float car_yaw, float car_pitch, uint8_t grade)
{
	vision_tx_data.enemy_color = enemy_color;
	vision_tx_data.car_yaw = car_yaw;
	vision_tx_data.car_pitch = car_pitch;
	vision_tx_data.grade = grade;
}

Vision_Tx_s *VisionGetTxData(void)
{
	return &vision_tx_data;
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

static void VisionDecodeFromBuffer(const uint8_t *buffer, uint16_t length)
{
	if (VisionUnpackRxFrame(buffer, length, &vision_rx_data))
	{
		// DaemonReload(vision_daemon);
	}
}

static void VisionDecodeCallback(void)
{
	VisionDecodeFromBuffer(vision_usart_instance->recv_buff, VISION_RECV_BUFF_SIZE);
}

Vision_Rx_s *VisionInit(UART_HandleTypeDef *uart_handle)
{
	USART_Init_Config_s usart_config;
	// Daemon_Init_Config_s daemon_config;

	usart_config.module_callback = VisionDecodeCallback;
	usart_config.recv_buff_size = VISION_RECV_BUFF_SIZE;
	usart_config.usart_handle = uart_handle;
	vision_usart_instance = USARTRegister(&usart_config);

	// daemon_config.callback = VisionOfflineCallback;
	// daemon_config.owner_id = vision_usart_instance;
	// daemon_config.reload_count = 10;
	// vision_daemon = DaemonRegister(&daemon_config);

	return &vision_rx_data;
}

void VisionSend(void)
{
	static uint8_t send_buffer[VISION_READ_FRAME_LEN];
	static uint16_t send_len;

	VisionPackTxFrame(&vision_tx_data, send_buffer, &send_len);
	USARTSend(vision_usart_instance, send_buffer, send_len, USART_TRANSFER_DMA);
}

#endif

#ifdef VISION_USE_VCP

#include "bsp_usb.h"

static uint8_t *vision_vcp_rx_buffer;

static void VisionDecodeCallback(uint16_t recv_len)
{
	if (VisionUnpackRxFrame(vision_vcp_rx_buffer, recv_len, &vision_rx_data))
	{
		// DaemonReload(vision_daemon);
	}
}

Vision_Rx_s *VisionInit(UART_HandleTypeDef *uart_handle)
{
	USB_Init_Config_s usb_config;
	// Daemon_Init_Config_s daemon_config;

	UNUSED(uart_handle);
	usb_config.tx_cbk = NULL;
	usb_config.rx_cbk = VisionDecodeCallback;
	vision_vcp_rx_buffer = USBInit(usb_config);

	// daemon_config.callback = VisionOfflineCallback;
	// daemon_config.owner_id = NULL;
	// daemon_config.reload_count = 5;
	// vision_daemon = DaemonRegister(&daemon_config);

	return &vision_rx_data;
}

void VisionSend(void)
{
	static uint8_t send_buffer[VISION_READ_FRAME_LEN];
	static uint16_t send_len;

	VisionPackTxFrame(&vision_tx_data, send_buffer, &send_len);
	USBTransmit(send_buffer, send_len);
}

#endif

void VisionTask(void)
{
	attitude_t *ins_data = INS_Init();

	VisionSetTxData(
		vision_tx_data.enemy_color,
		ins_data->Yaw * DEGREE_2_RAD,
		ins_data->Pitch * DEGREE_2_RAD,
		vision_tx_data.grade);
	VisionSend();
}
