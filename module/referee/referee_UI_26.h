#ifndef REFEREE_UI_26_H
#define REFEREE_UI_26_H

#include "stdarg.h"
#include "stdint.h"
#include "referee_protocol_26.h"
#include "referee_26.h"

#pragma pack(1) // 按1字节对齐

/* 交互数据头 (0x0301数据段的前6字节) */
typedef struct
{
   uint16_t data_cmd_id;
   uint16_t sender_ID;
   uint16_t receiver_ID;
} interaction_header_t;

/* 交互数据ID */
typedef enum
{
   UI_Data_ID_Del = 0x100,
   UI_Data_ID_Draw1 = 0x101,
   UI_Data_ID_Draw2 = 0x102,
   UI_Data_ID_Draw5 = 0x103,
   UI_Data_ID_Draw7 = 0x104,
   UI_Data_ID_DrawChar = 0x110,
} Interactive_Data_ID_e;

/* 交互数据长度 */
typedef enum
{
   Interactive_Data_LEN_Head = 6,
   UI_Operate_LEN_Del = 2,
   UI_Operate_LEN_PerDraw = 15,
   UI_Operate_LEN_DrawChar = 15 + 30,
} Interactive_Data_Length_e;

/* 此处的定义只与UI绘制有关 */
typedef struct
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   interaction_header_t datahead;
   uint8_t Delete_Operate; // 删除操作
   uint8_t Layer;
   uint16_t frametail;
} UI_delete_t;

typedef struct
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   interaction_header_t datahead;
   uint16_t frametail;
} UI_GraphReFresh_t;

typedef struct
{
   xFrameHeader FrameHeader;
   uint16_t CmdID;
   interaction_header_t datahead;
   ext_client_custom_character_t String_Data;
   uint16_t frametail;
} UI_CharReFresh_t; // 打印字符串数据

#pragma pack()

void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer);

void UILineDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void UIRectangleDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void UICircleDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                  uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

void UIOvalDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y);

void UIArcDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
               uint32_t end_x, uint32_t end_y);

void UIFloatDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                 uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float);

void UIIntDraw(interaction_figure_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer);

void UICharDraw(ext_client_custom_character_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...);

void UIGraphRefresh(referee_id_t *_id, int cnt, ...);

void UICharRefresh(referee_id_t *_id, ext_client_custom_character_t string_Data);

#endif
