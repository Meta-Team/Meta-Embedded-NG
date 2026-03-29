/**
 * @file vision_protocol_26.c
 * @brief Z_LION_AutoAim2025 串口协议(嵌入式侧)封装实现
 */

#include "vision_protocol_26.h"
#include "crc_ref.h"
#include <string.h>

enum
{
	SEND_IDX_SOF = 0,
	SEND_IDX_LEN_L = 1,
	SEND_IDX_LEN_H = 2,
	SEND_IDX_SEQ = 3,
	SEND_IDX_CRC8 = 4,
	SEND_IDX_CMD_L = 5,
	SEND_IDX_CMD_H = 6,
	SEND_IDX_PAYLOAD_START = 7,
	SEND_IDX_PAYLOAD = 8
};

enum
{
	PAYLOAD_IDX_PITCH = 0,
	PAYLOAD_IDX_YAW = 4,
	PAYLOAD_IDX_YAW_SPEED = 8,
	PAYLOAD_IDX_CAN_SHOOT = 12,
	PAYLOAD_IDX_MOVE_STATE = 14,
	PAYLOAD_IDX_IF_GET = 16,
	PAYLOAD_IDX_ENEMY_KIND = 18,
	PAYLOAD_IDX_ENEMY_X = 19,
	PAYLOAD_IDX_ENEMY_Y = 21,
	PAYLOAD_IDX_END = 23,
	PAYLOAD_LEN = 25,
};

static uint8_t VisionReadU8LE(const uint8_t *buffer)
{
	return (uint8_t)(buffer[0]);
}

static int8_t VisionReadI8LE(const uint8_t *buffer)
{
	return (int8_t)(buffer[0]);
}

static uint16_t VisionReadU16LE(const uint8_t *buffer)
{
	return (uint16_t)(buffer[0] | ((uint16_t)buffer[1] << 8));
}

static int16_t VisionReadI16LE(const uint8_t *buffer)
{
	return (int16_t)(buffer[0] | ((uint16_t)buffer[1] << 8));
}

static uint32_t VisionReadU32LE(const uint8_t *buffer)
{
	uint32_t value;
	memcpy(&value, buffer, sizeof(value));
	return value;
}

static int32_t VisionReadI32LE(const uint8_t *buffer)
{
	int32_t value;
	memcpy(&value, buffer, sizeof(value));
	return value;
}

static float VisionReadF32LE(const uint8_t *buffer)
{
	float value;
	memcpy(&value, buffer, sizeof(value));
	return value;
}

void VisionPackTxFrame(const Vision_Tx_Frame_s *frame, uint8_t *tx_buf, uint16_t *tx_len)
{
	uint16_t crc16;
	tx_buf[0] = VISION_READ_FRAME_HEAD;
	tx_buf[1] = frame->enemy_color;
	memcpy(&tx_buf[2], &frame->car_yaw, sizeof(float));
	memcpy(&tx_buf[6], &frame->car_pitch, sizeof(float));
	tx_buf[10] = frame->grade;

	crc16 = Get_CRC16_Check_Sum(tx_buf, VISION_READ_FRAME_LEN - 2u, 0xffffu);
	tx_buf[11] = (uint8_t)(crc16 & 0xffu);
	tx_buf[12] = (uint8_t)((crc16 >> 8) & 0xffu);

	*tx_len = VISION_READ_FRAME_LEN;
}

uint8_t VisionUnpackRxFrame(const uint8_t *rx_buf, uint16_t rx_len, Vision_Rx_Frame_s *frame)
{
	uint16_t offset;
	for (offset = 0; (uint32_t)offset + VISION_SEND_FRAME_LEN <= rx_len; offset++)
	{
		const uint8_t *p = rx_buf + offset;
		uint16_t data_len;
		uint16_t cmd_id;
		uint16_t crc16;
		uint16_t crc16_expect;

		if (p[SEND_IDX_SOF] != VISION_SEND_FRAME_SOF)
			continue;

		if (Get_CRC8_Check_Sum(p, SEND_IDX_CRC8, 0xffu) != p[SEND_IDX_CRC8])
			continue;

		data_len = VisionReadU16LE(&p[SEND_IDX_LEN_L]);
		if (data_len != PAYLOAD_LEN)
			continue;

		cmd_id = VisionReadU16LE(&p[SEND_IDX_CMD_L]);
		if (cmd_id != VISION_SEND_FRAME_CMD_ID)
			continue;

		if (p[SEND_IDX_PAYLOAD_START] != VISION_SEND_PAYLOAD_START ||
			p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_END] != VISION_SEND_PAYLOAD_END)
			continue;

		crc16 = Get_CRC16_Check_Sum(p, VISION_SEND_FRAME_LEN - 2u, 0xffffu);
		crc16_expect = VisionReadU16LE(p + VISION_SEND_FRAME_LEN - 2u);
		if (crc16 != crc16_expect)
			continue;

		frame->pitch = VisionReadF32LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_PITCH]);
		frame->yaw = VisionReadF32LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_YAW]);
		frame->yaw_speed = VisionReadF32LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_YAW_SPEED]);
		frame->can_shoot = VisionReadI16LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_CAN_SHOOT]);
		frame->move_state = VisionReadI16LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_MOVE_STATE]);
		frame->if_get = VisionReadI16LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_IF_GET]);
		frame->enemy_kind = p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_ENEMY_KIND];
		frame->enemy_x = VisionReadI16LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_ENEMY_X]);
		frame->enemy_y = VisionReadI16LE(&p[SEND_IDX_PAYLOAD + PAYLOAD_IDX_ENEMY_Y]);
		return 1u;
	}
	return 0u;
}
