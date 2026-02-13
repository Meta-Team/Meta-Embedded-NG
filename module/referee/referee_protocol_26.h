/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_26H
#define referee_protocol_26H

#include "stdint.h"

/****************************宏定义部分****************************/

#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0
#define Robot_Blue 1

// RoboMaster 2026 学生机器人间交互(0x0301)的数据段最大为 112 字节
// 注意：0x0301 数据段为变长，长度以帧头 DataLength 为准
#define REFEREE_INTERACTIVE_DATA_MAX_LEN 112

#pragma pack(1)

/****************************通信协议格式****************************/

/* 通信协议格式偏移，枚举类型,代替#define声明 */
typedef enum
{
    FRAME_HEADER_Offset = 0,
    CMD_ID_Offset = 5,
    DATA_Offset = 7,
} JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum
{
    LEN_HEADER = 5, // 帧头长
    LEN_CMDID = 2,	// 命令码长度
    LEN_TAIL = 2,	// 帧尾CRC16
    LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

/****************************帧头****************************/

/* 帧头偏移 */
typedef enum
{
    SOF = 0,		 // 起始位
    DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
    SEQ = 3,		 // 包序号
    CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
    uint8_t SOF;
    uint16_t DataLength;
    uint8_t Seq;
    uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 (RoboMaster 2026) */
typedef enum
{
    // 常规链路
    ID_game_status = 0x0001,               // 比赛状态数据
    ID_game_result = 0x0002,               // 比赛结果数据
    ID_game_robot_HP = 0x0003,             // 机器人血量数据
    ID_event_data = 0x0101,                // 场地事件数据
    ID_referee_warning = 0x0104,           // 裁判警告数据
    ID_dart_info = 0x0105,                 // 飞镖发射相关数据
    ID_robot_status = 0x0201,              // 机器人性能体系数据
    ID_power_heat_data = 0x0202,           // 实时底盘缓冲能量和射击热量数据
    ID_robot_pos = 0x0203,                 // 机器人位置数据
    ID_buff = 0x0204,                      // 机器人增益和底盘能量数据
    ID_hurt_data = 0x0206,                 // 伤害状态数据
    ID_shoot_data = 0x0207,                // 实时射击数据
    ID_projectile_allowance = 0x0208,      // 允许发弹量
    ID_rfid_status = 0x0209,               // RFID模块状态
    ID_dart_client_cmd = 0x020A,           // 飞镖选手端指令数据
    ID_ground_robot_position = 0x020B,     // 地面机器人位置数据
    ID_radar_mark_data = 0x020C,           // 雷达标记进度数据
    ID_sentry_info = 0x020D,               // 哨兵自主决策信息同步
    ID_radar_info = 0x020E,                // 雷达自主决策信息同步
    ID_robot_interaction_data = 0x0301,    // 机器人交互数据
    ID_map_command = 0x0303,               // 选手端小地图交互数据
    ID_map_robot_data = 0x0305,            // 选手端小地图接收雷达数据
    ID_map_data = 0x0307,                  // 选手端小地图接收路径数据
    ID_custom_info = 0x0308,               // 选手端小地图接收机器人数据

    // 图传链路
    ID_custom_robot_data = 0x0302,         // 机器人接收自定义控制器数据
    ID_robot_custom_data = 0x0309,         // 自定义控制器接收机器人数据
    ID_robot_custom_data_2 = 0x0310,       // 机器人发送给自定义客户端的数据
    ID_robot_custom_data_3 = 0x0311,       // 自定义客户端发送给机器人的自定义指令

    // 雷达无线链路
    ID_radar_enemy_position = 0x0A01,      // 对方机器人的位置坐标
    ID_radar_enemy_hp = 0x0A02,            // 对方机器人的血量信息
    ID_radar_enemy_projectile = 0x0A03,    // 对方机器人的剩余发弹量信息
    ID_radar_enemy_state = 0x0A04,         // 对方队伍的宏观状态信息
    ID_radar_enemy_effect = 0x0A05,        // 对方各机器人当前增益效果
    ID_radar_enemy_password = 0x0A06,      // 对方干扰波密钥

    // 非链路
    ID_custom_client_data = 0x0306,        // 自定义控制器与选手端交互数据
} CmdID_e;

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
typedef enum
{
    // 常规链路
    LEN_game_status = 11,                  // 0x0001 比赛状态数据
    LEN_game_result = 1,                   // 0x0002 比赛结果数据
    LEN_game_robot_HP = 16,                // 0x0003 机器人血量数据
    LEN_event_data = 4,                    // 0x0101 场地事件数据
    LEN_referee_warning = 3,               // 0x0104 裁判警告数据
    LEN_dart_info = 3,                     // 0x0105 飞镖发射相关数据
    LEN_robot_status = 13,                 // 0x0201 机器人性能体系数据
    LEN_power_heat_data = 14,              // 0x0202 实时底盘缓冲能量和射击热量数据
    LEN_robot_pos = 12,                    // 0x0203 机器人位置数据
    LEN_buff = 8,                          // 0x0204 机器人增益和底盘能量数据
    LEN_hurt_data = 1,                     // 0x0206 伤害状态数据
    LEN_shoot_data = 7,                    // 0x0207 实时射击数据
    LEN_projectile_allowance = 8,          // 0x0208 允许发弹量
    LEN_rfid_status = 5,                   // 0x0209 RFID模块状态
    LEN_dart_client_cmd = 6,               // 0x020A 飞镖选手端指令数据
    LEN_ground_robot_position = 40,        // 0x020B 地面机器人位置数据
    LEN_radar_mark_data = 2,               // 0x020C 雷达标记进度数据
    LEN_sentry_info = 6,                   // 0x020D 哨兵自主决策信息同步
    LEN_radar_info = 1,                    // 0x020E 雷达自主决策信息同步
    LEN_robot_interaction_data_header = 6, // 0x0301 机器人交互数据段头
    LEN_robot_interaction_data_max = LEN_robot_interaction_data_header + REFEREE_INTERACTIVE_DATA_MAX_LEN,
    LEN_map_command = 12,                  // 0x0303 选手端小地图交互数据
    LEN_map_robot_data = 24,               // 0x0305 选手端小地图接收雷达数据
    LEN_map_data = 103,                    // 0x0307 选手端小地图接收路径数据
    LEN_custom_info = 34,                  // 0x0308 选手端小地图接收机器人数据

    // 图传链路
    LEN_custom_robot_data = 30,            // 0x0302 机器人接收自定义控制器数据
    LEN_robot_custom_data = 30,            // 0x0309 自定义控制器接收机器人数据
    LEN_robot_custom_data_2 = 300,         // 0x0310 机器人发送给自定义客户端的数据
    LEN_robot_custom_data_3 = 30,          // 0x0311 自定义客户端发送给机器人的自定义指令

    // 雷达无线链路
    LEN_radar_enemy_position = 24,         // 0x0A01 对方机器人的位置坐标
    LEN_radar_enemy_hp = 12,               // 0x0A02 对方机器人的血量信息
    LEN_radar_enemy_projectile = 10,       // 0x0A03 对方机器人的剩余发弹量信息
    LEN_radar_enemy_state = 8,             // 0x0A04 对方队伍的宏观状态信息
    LEN_radar_enemy_effect = 36,           // 0x0A05 对方各机器人当前增益效果
    LEN_radar_enemy_password = 6,          // 0x0A06 对方干扰波密钥

    // 非链路
    LEN_custom_client_data = 8,            // 0x0306 自定义控制器与选手端交互数据

} JudgeDataLength_e;

/****************************接收数据的详细说明****************************/
/****************************常规链路数据说明****************************/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct
{
    uint8_t winner;
} game_result_t;

/* ID: 0x0003  Byte:  16    机器人血量数据*/
typedef struct
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
    uint32_t event_data;
} event_data_t;

/* ID: 0x0104  Byte:  3    裁判警告数据 */
typedef struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

/* ID: 0x0105  Byte:  3    飞镖发射相关数据 */
typedef struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;

/* ID: 0x0201  Byte: 13    机器人性能体系数据 */
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

/* ID: 0x0202  Byte: 14    实时底盘缓冲能量和射击热量数据 */
typedef struct
{
    uint16_t reserved0;
    uint16_t reserved1;
    float reserved2;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/* ID: 0x0203  Byte: 12    机器人位置数据 */
typedef struct
{
    float x;
    float y;
    float angle;
} robot_pos_t;

/* ID: 0x0204  Byte:  8    机器人增益和底盘能量数据 */
typedef struct
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

/* ID: 0x0208 Byte: 8 允许发弹量 */
typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* ID: 0x0209 Byte: 5 RFID模块状态 */
typedef struct
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} rfid_status_t;

/* ID: 0x020A Byte: 6 飞镖选手端指令数据 */
typedef struct
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

/* ID: 0x020B Byte: 40 地面机器人位置数据 */
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved0;
    float reserved1;
} ground_robot_position_t;

/* ID: 0x020C Byte: 2 雷达标记进度数据 */
typedef struct
{
    uint16_t mark_progress;
} radar_mark_data_t;

/* ID: 0x020D Byte: 6 哨兵自主决策信息同步 */
typedef struct
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

/* ID: 0x020E Byte: 1 雷达自主决策信息同步 */
typedef struct
{
    uint8_t radar_info;
} radar_info_t;


/* ID: 0x0301 Byte: max118 机器人交互数据 */
typedef struct
{
  uint16_t data_cmd_id;
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[];
} robot_interaction_data_t;

// 以下为0x0301子内容ID数据说明
/* 子内容ID: 0x0200-0x02FF Byte: max118 机器人之间通信（未实现） */

/* 子内容ID: 0x0100 Byte: 2 选手端删除图层 */
typedef struct
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

typedef enum
{
    delete_type_NoOperate = 0,
    delete_type_Layer = 1,
    delete_type_ALL = 2,
} delete_type_e;

/* 子内容ID: 0x0101 Byte: 15 选手端绘制一个图形 */
typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_type:3;
    uint32_t figure_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} interaction_figure_t;

typedef enum
{
    operate_type_NoOperate = 0,
    operate_type_ADD = 1,
    operate_type_Change = 2,
    operate_type_Del = 3,
} operate_type_e;

typedef enum
{
    figure_type_Line = 0,		// 直线
    figure_type_Rectangle = 1, // 矩形
    figure_type_Circle = 2,	// 整圆
    figure_type_Ellipse = 3,	// 椭圆
    figure_type_Arc = 4,		// 圆弧
    figure_type_Float = 5,		// 浮点型
    figure_type_Int = 6,		// 整形
    figure_type_Char = 7,		// 字符型

} figure_type_e;

typedef enum
{
    color_Main = 0, // 红蓝主色
    color_Yellow = 1,
    color_Green = 2,
    color_Orange = 3,
    color_Purplish_red = 4, // 紫红色
    color_Pink = 5,
    color_Cyan = 6, // 青色
    color_Black = 7,
    color_White = 8,

} color_e;

/* 子内容ID: 0x0102 Byte: 30 选手端绘制两个图形 */

typedef struct
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/* 子内容ID: 0x0103 Byte: 75 选手端绘制五个图形 */

typedef struct
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

/* 子内容ID: 0x0104 Byte: 105 选手端绘制七个图形 */

typedef struct
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

/* 子内容ID: 0x0110 Byte: 45 选手端绘制字符图形 */

typedef struct
{
    interaction_figure_t  graphic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

/* 子内容ID: 0x0120 Byte: 4 哨兵自主决策指令 */

typedef struct
{
    uint32_t sentry_cmd;
} sentry_cmd_t;

/* 子内容ID: 0x0121 Byte: 8 雷达自主决策指令 */

typedef struct
{
    uint8_t radar_cmd;
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
} radar_cmd_t;


/* ID: 0x0303 Byte: 12 选手端小地图交互数据 */
typedef struct
{
    float target_position_x;    // 目标X坐标
    float target_position_y;    // 目标Y坐标
    uint8_t cmd_keyboard;       // 云台手按下的键值
    uint8_t target_robot_id;    // 对方机器人ID
    uint16_t cmd_source;        // 信息来源ID
} map_command_t;


/* ID: 0x0305 Byte: 24 选手端小地图接收雷达数据 */
typedef struct
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t reserved0;
    uint16_t reserved1;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;

/* ID: 0x0307 Byte: 103 选手端小地图接收路径数据 */
typedef struct
{
    uint8_t intention;          // 意图 1:攻击 2:防守 3:移动
    uint16_t start_position_x;  // 起始点X
    uint16_t start_position_y;  // 起始点Y
    int8_t delta_x[49];         // X轴增量数组
    int8_t delta_y[49];         // Y轴增量数组
    uint16_t sender_id;         // 发送者ID
} map_data_t;

/* ID: 0x0308 Byte: 34 选手端小地图接收机器人数据 */
typedef struct
{
    uint16_t sender_id;      // 发送者机器人ID
    uint16_t receiver_id;    // 接收者机器人ID
    uint8_t user_data[30];   // 用户自定义数据
} custom_info_t;


/****************************图传链路数据说明****************************/

/* ID: 0x0302 Byte: 30 机器人接收自定义控制器数据 */
typedef struct
{
    uint8_t data[30];
} custom_robot_data_t;

/* ID: 0x0309 Byte: 30 自定义控制器接收机器人数据 */
typedef struct
{
    uint8_t data[30];
} robot_custom_data_t;

/* ID: 0x0310 Byte: 300 机器人发送给自定义客户端的数据 */
typedef struct
{
    uint8_t data[300];
} robot_custom_data_2_t;

/* ID: 0x0311 Byte: 30 自定义客户端发送给机器人的自定义指令 */
typedef struct
{
    uint8_t data[30];
} robot_custom_data_3_t;


/****************************雷达无线链路数据说明****************************/

// 0x0A01 对方机器人的位置坐标
typedef struct
{
    uint16_t enemy_hero_x;
    uint16_t enemy_hero_y;
    uint16_t enemy_engineer_x;
    uint16_t enemy_engineer_y;
    uint16_t enemy_infantry3_x;
    uint16_t enemy_infantry3_y;
    uint16_t enemy_infantry4_x;
    uint16_t enemy_infantry4_y;
    uint16_t enemy_aerial_x;
    uint16_t enemy_aerial_y;
    uint16_t enemy_sentry_x;
    uint16_t enemy_sentry_y;
} radar_enemy_position_t;

// 0x0A02 对方机器人的血量信息
typedef struct
{
    uint16_t enemy_1_robot_HP;
    uint16_t enemy_2_robot_HP;
    uint16_t enemy_3_robot_HP;
    uint16_t enemy_4_robot_HP;
    uint16_t reserved;
    uint16_t enemy_7_robot_HP;
} radar_enemy_hp_t;

// 0x0A03 对方机器人的剩余发弹量信息
typedef struct
{
    uint16_t enemy_1_projectile;
    uint16_t enemy_3_projectile;
    uint16_t enemy_4_projectile;
    uint16_t enemy_6_projectile;
    uint16_t enemy_7_projectile;
} radar_enemy_projectile_t;

// 0x0A04 对方队伍的宏观状态信息
typedef struct
{
    uint16_t enemy_remain_coin;
    uint16_t enemy_total_coin;
    uint32_t enemy_state;
} radar_enemy_state_t;

// 0x0A05 对方各机器人当前增益效果
typedef struct
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} radar_enemy_effect_unit_t;

typedef struct
{
    radar_enemy_effect_unit_t hero;
    radar_enemy_effect_unit_t engineer;
    radar_enemy_effect_unit_t infantry3;
    radar_enemy_effect_unit_t infantry4;
    radar_enemy_effect_unit_t sentry;
    uint8_t sentry_state;
} radar_enemy_effect_t;

// 0x0A06 对方干扰波密钥
typedef struct
{
    uint8_t password[6];
} radar_enemy_password_t;

/****************************非链路数据说明****************************/

// 0x0306 自定义控制器与选手端交互数据
typedef struct
{
    uint16_t key_value;          // 键盘值
    uint16_t x_position : 12;    // X位置（12位）
    uint16_t mouse_left : 4;     // 左键（4位）
    uint16_t y_position : 12;    // Y位置（12位）
    uint16_t mouse_right : 4;    // 右键（4位）
    uint16_t reserved;           // 保留字段
} custom_client_data_t;

/* 机器人id */
typedef enum
{
    // 红方机器人ID
    robotID_RHero = 1,
    robotID_REngineer = 2,
    robotID_RInfantry1 = 3,
    robotID_RInfantry2 = 4,
    robotID_RInfantry3 = 5,
    robotID_RAerial = 6,
    robotID_RSentry = 7,
    robotID_RRadar = 9,
    // 蓝方机器人ID
    robotID_BHero = 101,
    robotID_BEngineer = 102,
    robotID_BInfantry1 = 103,
    robotID_BInfantry2 = 104,
    robotID_BInfantry3 = 105,
    robotID_BAerial = 106,
    robotID_BSentry = 107,
    robotID_BRadar = 109,
} robot_ID_e;

/* 选手端ID */
typedef enum
{
    // 红方选手端ID
    clientID_RHero = 0x0101,        // 红方英雄机器人选手端
    clientID_REngineer = 0x0102,    // 红方工程机器人选手端
    clientID_RInfantry1 = 0x0103,   // 红方步兵机器人选手端（机器人ID 3）
    clientID_RInfantry2 = 0x0104,   // 红方步兵机器人选手端（机器人ID 4）
    clientID_RInfantry3 = 0x0105,   // 红方步兵机器人选手端（机器人ID 5）
    clientID_RAerial = 0x0106,      // 红方空中机器人选手端
    // 蓝方选手端ID
    clientID_BHero = 0x0165,        // 蓝方英雄机器人选手端
    clientID_BEngineer = 0x0166,    // 蓝方工程机器人选手端
    clientID_BInfantry1 = 0x0167,   // 蓝方步兵机器人选手端（机器人ID 3）
    clientID_BInfantry2 = 0x0168,   // 蓝方步兵机器人选手端（机器人ID 4）
    clientID_BInfantry3 = 0x0169,   // 蓝方步兵机器人选手端（机器人ID 5）
    clientID_BAerial = 0x016A,      // 蓝方空中机器人选手端
    // 裁判系统服务器
    clientID_Referee = 0x8080,      // 裁判系统服务器（哨兵和雷达自主决策指令）
} client_ID_e;

#pragma pack()

#endif
