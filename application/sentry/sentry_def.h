#ifndef SENTRY_DEF_H
#define SENTRY_DEF_H

#include <stdint.h>

/*
 * sentry（哨兵）专属参数与配置。
 * 通用消息类型请见 application/robot_cmd_def.h，本文件只放机器人相关参数。
 * 本文件不应被直接 include，统一通过 application/robot_def.h 选择器引入。
 */

#define ONE_BOARD

#define VISION_USE_VCP
#define DEBUG

/* 云台参数 */
#define YAW_CHASSIS_ALIGN_DEG 197   // 云台和底盘对齐指向相同方向时的电机角度值,若对云台有机械改动需要修改
#define PITCH_HORIZON_RAD -1        // 云台处于水平位置时电机弧度值,若对云台有机械改动需要修改
#define PITCH_MAX_RAD 0.5           // 云台竖直方向最大角度 (电机编码弧度)
#define PITCH_MIN_RAD -1.5          // 云台竖直方向最小角度 (电机编码弧度)
#define PITCH_MAX_VEL 1             // 云台竖直方向最大速度 (电机编码弧度)
#define PITCH_MIN_VEL -1           // 云台竖直方向最小速度 (电机编码弧度)
#define PITCH_MAX_ECD_DEG 200.0f    // 云台pitch最大仰角对应的电机编码器角度(度),需实测标定
#define PITCH_MIN_ECD_DEG 140.0f    // 云台pitch最大俯角对应的电机编码器角度(度),需实测标定

/* 发射参数 */
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER (54.74f / 25.16f)  // 3508拨盘电机的减速比,未装减速箱,拨盘齿轮减速比
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量

/* 底盘参数 */
#define CHASSIS_RY (326.0f / 2.0f)     // 纵向轮距(前进后退方向),单位为mm(毫米)
#define CHASSIS_RX (326.0f / 2.0f)     // 横向轮距(左右平移方向),单位为mm(毫米)
#define CHASSIS_RX_M (CHASSIS_RX / 1000.0f)  // 转换为米
#define CHASSIS_RY_M (CHASSIS_RY / 1000.0f)  // 转换为米
#define CENTER_GIMBAL_OFFSET_X 0
#define CENTER_GIMBAL_OFFSET_Y 0
#define RADIUS_WHEEL 58.0f           // 轮子半径,单位为mm(毫米)
#define RADIUS_WHEEL_M (RADIUS_WHEEL / 1000.0f)  // 转换为米
#define PERIMETER_WHEEL (RADIUS_WHEEL_M * 2 * PI) // 轮子周长
#define REDUCTION_RATIO_WHEEL 19.0f  // 电机减速比
#define RF_WHEEL_POS_OFFSET_ANGLE 102.5f  // 右前轮舵面初始位置偏移角度(度)
#define RB_WHEEL_POS_OFFSET_ANGLE 12.5f  // 右后轮
#define LB_WHEEL_POS_OFFSET_ANGLE 137.5f  // 左后轮
#define LF_WHEEL_POS_OFFSET_ANGLE 145.0f  // 左前轮

#endif // !SENTRY_DEF_H
