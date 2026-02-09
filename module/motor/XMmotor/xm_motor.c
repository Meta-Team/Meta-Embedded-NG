/**
 * @file xm_motor.c
 * @brief 小米电机驱动实现 (XiaoMi/CyberGear Motor)
 * @note  基于CAN 2.0扩展帧协议,波特率1Mbps
 *        目前只实现运控模式(MIT模式)
 */

#include "xm_motor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

static uint8_t idx;
static XMMotorInstance *xm_motor_instance[XM_MOTOR_CNT];
static osThreadId xm_task_handle[XM_MOTOR_CNT];

/* ----------------------- 静态辅助函数 ----------------------- */

/**
 * @brief 将浮点数映射为无符号整数
 * @param x 输入浮点数
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 输出位数
 * @return uint16_t 映射后的整数值
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 将无符号整数映射为浮点数
 * @param x_int 输入整数
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 输入位数
 * @return float 映射后的浮点数值
 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 构造扩展帧ID
 * @param mode 通信类型
 * @param data 数据区2 (bit23~8)
 * @param id 目标电机CAN_ID (bit7~0)
 * @return uint32_t 29位扩展帧ID
 */
static uint32_t XMMotorBuildCanId(uint8_t mode, uint16_t data, uint8_t id)
{
    uint32_t ext_id = 0;
    ext_id |= ((uint32_t)mode & 0x1F) << 24;  // bit28~24: 通信类型
    ext_id |= ((uint32_t)data & 0xFFFF) << 8; // bit23~8: 数据区2
    ext_id |= ((uint32_t)id & 0xFF);          // bit7~0: 目标电机CAN_ID
    return ext_id;
}

/**
 * @brief 解析扩展帧ID
 * @param ext_id 29位扩展帧ID
 * @param info 输出的ID信息结构体指针
 */
static void XMMotorParseCanId(uint32_t ext_id, XM_CanIdInfo_s *info)
{
    info->mode = (ext_id >> 24) & 0x1F;
    info->data = (ext_id >> 8) & 0xFFFF;
    info->id = ext_id & 0xFF;
}

/**
 * @brief 发送电机模式命令(使能/停止/零位校准)
 * @param motor 电机实例指针
 * @param mode 命令类型
 */
static void XMMotorSendModeCmd(XMMotorInstance *motor, XM_Comm_Type_e mode)
{
    uint32_t ext_id;
    
    switch (mode)
    {
        case XM_CMD_MOTOR_ENABLE:
            // 通信类型3: 电机使能运行
            // 29位ID: mode=3, data=主机CAN_ID, id=目标电机CAN_ID
            ext_id = XMMotorBuildCanId(XM_CMD_MOTOR_ENABLE, motor->master_id, motor->motor_id);
            break;
        case XM_CMD_MOTOR_STOP:
            // 通信类型4: 电机停止运行
            // 正常运行时data区需清零, Byte[0]=1时清故障
            ext_id = XMMotorBuildCanId(XM_CMD_MOTOR_STOP, motor->master_id, motor->motor_id);
            break;
        case XM_CMD_SET_ZERO:
            // 通信类型6: 设置电机机械零位
            // Byte[0]=1
            ext_id = XMMotorBuildCanId(XM_CMD_SET_ZERO, motor->master_id, motor->motor_id);
            break;
        default:
            return;
    }
    
    // 设置扩展帧ID
    CANSetTxId(motor->motor_can_instance, ext_id);
    
    // 清空发送缓冲区
    memset(motor->motor_can_instance->tx_buff, 0, 8);
    
    // 对于设置零位命令, Byte[0]=1
    if (mode == XM_CMD_SET_ZERO)
    {
        motor->motor_can_instance->tx_buff[0] = 1;
    }
    
    CANTransmit(motor->motor_can_instance, 1);
}

/**
 * @brief CAN接收回调函数,解析电机反馈数据
 * @param motor_can CAN实例指针
 * @note  由于多个电机可能共享相同的CAN接收过滤器,此函数会遍历所有XM电机实例
 */
static void XMMotorDecode(CANInstance *motor_can)
{
    uint8_t *rxbuff = motor_can->rx_buff;
    uint32_t rx_ext_id = motor_can->last_rx_identifier;
    
    // 解析扩展帧ID
    XM_CanIdInfo_s rx_id_info;
    XMMotorParseCanId(rx_ext_id, &rx_id_info);
    
    // 检查是否为电机反馈帧 (通信类型2)
    if (rx_id_info.mode != XM_CMD_MOTOR_FEEDBACK)
    {
        return;
    }
    
    // 从扩展帧ID中提取电机ID
    // rx_id_info.data = bit8~23:
    //   bit8~15(即data的低8位): 电机CAN ID
    uint8_t motor_id = (rx_id_info.data >> 0) & 0xFF;
    
    // 遍历所有XM电机实例,找到匹配的电机
    for (size_t i = 0; i < idx; i++)
    {
        XMMotorInstance *motor = xm_motor_instance[i];
        if (motor == NULL) continue;
        
        // 检查电机ID和CAN句柄是否匹配
        if (motor->motor_id == motor_id && 
            motor->motor_can_instance->can_handle == motor_can->can_handle)
        {
            XM_Motor_Measure_s *measure = &(motor->measure);
            
            // 从扩展帧ID中提取状态信息
            measure->id = motor_id;
            measure->fault = (rx_id_info.data >> 8) & 0x3F;
            measure->mode = (XM_Motor_Mode_State_e)((rx_id_info.data >> 14) & 0x03);
            
            // 保存上一次角度值
            measure->last_position = measure->position;
            
            // 解析数据区
            // Byte0~1: 当前角度 [0~65535] -> (-4π ~ 4π)
            uint16_t tmp_pos = ((uint16_t)rxbuff[0] << 8) | rxbuff[1];
            measure->position = uint_to_float(tmp_pos, XM_P_MIN, XM_P_MAX, 16);
            
            // Byte2~3: 当前角速度 [0~65535] -> (-30rad/s ~ 30rad/s)
            uint16_t tmp_vel = ((uint16_t)rxbuff[2] << 8) | rxbuff[3];
            measure->velocity = uint_to_float(tmp_vel, XM_V_MIN, XM_V_MAX, 16);
            
            // Byte4~5: 当前力矩 [0~65535] -> (-12Nm ~ 12Nm)
            uint16_t tmp_torque = ((uint16_t)rxbuff[4] << 8) | rxbuff[5];
            measure->torque = uint_to_float(tmp_torque, XM_T_MIN, XM_T_MAX, 16);
            
            // Byte6~7: 当前温度, Temp(摄氏度)*10
            uint16_t tmp_temp = ((uint16_t)rxbuff[6] << 8) | rxbuff[7];
            measure->temperature = (float)tmp_temp / 10.0f;
            
            return; // 找到匹配的电机后返回
        }
    }
}

/**
 * @brief 电机丢失回调函数
 * @param motor_ptr 电机实例指针
 */
static void XMMotorLostCallback(void *motor_ptr)
{
    XMMotorInstance *motor = (XMMotorInstance *)motor_ptr;
    motor->lost_cnt++;
}

/* ----------------------- 公开接口函数 ----------------------- */

XMMotorInstance *XMMotorInit(Motor_Init_Config_s *config)
{
    XMMotorInstance *motor = (XMMotorInstance *)malloc(sizeof(XMMotorInstance));
    memset(motor, 0, sizeof(XMMotorInstance));
    
    // 保存电机设置
    motor->motor_settings = config->controller_setting_init_config;
    
    // 配置MIT模式的Kp和Kd
    // 从angle_PID.Kp获取位置Kp,从speed_PID.Kd获取阻尼Kd
    motor->Kp = config->controller_param_init_config.angle_PID.Kp;
    motor->Kd = config->controller_param_init_config.speed_PID.Kd;
    
    // 配置外部反馈来源
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    
    // 提取电机ID (从CAN配置的rx_id中)
    // 对于小米电机,使用扩展帧,电机ID应该单独配置
    // 这里假设rx_id的低8位为电机ID
    motor->motor_id = config->can_init_config.rx_id & 0xFF;
    motor->master_id = XM_MASTER_CAN_ID;
    
    // 配置CAN实例
    // 小米电机使用扩展帧格式
    config->can_init_config.id_type = CAN_ID_EXT;
    config->can_init_config.can_module_callback = XMMotorDecode;
    config->can_init_config.id = motor;
    
    // 设置接收ID和掩码
    // 对于反馈帧(通信类型2): 
    //   bit28~24=2, bit23~8=状态|电机ID, bit7~0=主机CAN_ID
    // 我们需要匹配: 通信类型=2, 且低8位=主机CAN_ID
    // 构造期望的接收ID: mode=2, data=任意, id=主机ID
    config->can_init_config.rx_id = XMMotorBuildCanId(XM_CMD_MOTOR_FEEDBACK, 0, motor->master_id & 0xFF);
    // 掩码: 只匹配bit28~24(通信类型)和bit7~0(主机ID)
    config->can_init_config.rx_id_mask = 0x1F0000FF;
    
    // 设置初始发送ID (运控模式)
    config->can_init_config.tx_id = XMMotorBuildCanId(XM_CMD_CONTROL_MODE, 
                                                       float_to_uint(0, XM_T_MIN, XM_T_MAX, 16),
                                                       motor->motor_id);
    
    motor->motor_can_instance = CANRegister(&config->can_init_config);
    
    // 使能电机
    motor->stop_flag = MOTOR_ENALBED;
    
    // 发送使能命令
    DWT_Delay(0.1);
    XMMotorSendModeCmd(motor, XM_CMD_MOTOR_ENABLE);
    DWT_Delay(0.1);
    
    // 校准零位(可选)
    // XMMotorCaliEncoder(motor);
    
    // 保存实例
    xm_motor_instance[idx++] = motor;
    
    LOGINFO("[xm_motor] XM Motor %d Init Success", motor->motor_id);
    
    return motor;
}

void XMMotorSetRef(XMMotorInstance *motor, float position, float velocity, float torque)
{
    motor->position_ref = position;
    motor->velocity_ref = velocity;
    motor->torque_ref = torque;
}

void XMMotorEnable(XMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
    XMMotorSendModeCmd(motor, XM_CMD_MOTOR_ENABLE);
}

void XMMotorStop(XMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
    XMMotorSendModeCmd(motor, XM_CMD_MOTOR_STOP);
}

void XMMotorCaliEncoder(XMMotorInstance *motor)
{
    XMMotorSendModeCmd(motor, XM_CMD_SET_ZERO);
    DWT_Delay(0.1);
}

void XMMotorOuterLoop(XMMotorInstance *motor, Closeloop_Type_e closeloop_type)
{
    motor->motor_settings.outer_loop_type = closeloop_type;
}

/**
 * @brief 电机控制任务 (每个电机实例一个任务)
 * @param argument 电机实例指针
 */
static void XMMotorTask(void const *argument)
{
    float pos_set, vel_set, torque_set;
    XMMotorInstance *motor = (XMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    XM_Motor_Send_s motor_send_mailbox;
    uint32_t ext_id;
    
    while (1)
    {
        // 获取参考值
        pos_set = motor->position_ref;
        vel_set = motor->velocity_ref;
        torque_set = motor->torque_ref;
        
        // 方向处理
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            pos_set *= -1;
            vel_set *= -1;
            torque_set *= -1;
        }
        
        // 限幅
        LIMIT_MIN_MAX(pos_set, XM_P_MIN, XM_P_MAX);
        LIMIT_MIN_MAX(vel_set, XM_V_MIN, XM_V_MAX);
        LIMIT_MIN_MAX(torque_set, XM_T_MIN, XM_T_MAX);
        
        // 填充发送数据
        motor_send_mailbox.position_des = float_to_uint(pos_set, XM_P_MIN, XM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(vel_set, XM_V_MIN, XM_V_MAX, 16);
        motor_send_mailbox.Kp = float_to_uint(motor->Kp, XM_KP_MIN, XM_KP_MAX, 16);
        motor_send_mailbox.Kd = float_to_uint(motor->Kd, XM_KD_MIN, XM_KD_MAX, 16);
        motor_send_mailbox.torque_des = float_to_uint(torque_set, XM_T_MIN, XM_T_MAX, 16);
        
        // 如果电机停止,所有输出清零
        if (motor->stop_flag == MOTOR_STOP)
        {
            motor_send_mailbox.position_des = float_to_uint(0, XM_P_MIN, XM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, XM_V_MIN, XM_V_MAX, 16);
            motor_send_mailbox.torque_des = float_to_uint(0, XM_T_MIN, XM_T_MAX, 16);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;
        }
        
        // 构造扩展帧ID
        // 通信类型1: 运控模式
        // 29位ID结构: mode=1, data=力矩(16bit), id=目标电机CAN_ID
        // 根据说明书样例: txCanIdEx.data = float_to_uint(torque, T_MIN, T_MAX, 16)
        ext_id = XMMotorBuildCanId(XM_CMD_CONTROL_MODE, motor_send_mailbox.torque_des, motor->motor_id);
        CANSetTxId(motor->motor_can_instance, ext_id);
        
        // 填充数据区 (8字节)
        // 根据说明书样例代码:
        // Byte0~1: 目标角度(高8位>>8, 低8位)
        // Byte2~3: 目标角速度(高8位>>8, 低8位)
        // Byte4~5: Kp(高8位>>8, 低8位)
        // Byte6~7: Kd(高8位>>8, 低8位)
        motor->motor_can_instance->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instance->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des & 0xFF);
        motor->motor_can_instance->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 8);
        motor->motor_can_instance->tx_buff[3] = (uint8_t)(motor_send_mailbox.velocity_des & 0xFF);
        motor->motor_can_instance->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp >> 8);
        motor->motor_can_instance->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kp & 0xFF);
        motor->motor_can_instance->tx_buff[6] = (uint8_t)(motor_send_mailbox.Kd >> 8);
        motor->motor_can_instance->tx_buff[7] = (uint8_t)(motor_send_mailbox.Kd & 0xFF);
        
        CANTransmit(motor->motor_can_instance, 1);
        
        osDelay(2);
    }
}

void XMMotorControlInit(void)
{
    char xm_task_name[8];
    
    if (!idx)
        return;
    
    for (size_t i = 0; i < idx; i++)
    {
        strcpy(xm_task_name, "xm");  // 每次循环重置任务名
        char xm_id_buff[2] = {0};
        __itoa(i, xm_id_buff, 10);
        strcat(xm_task_name, xm_id_buff);
        
        osThreadDef(xm_task_name, XMMotorTask, osPriorityNormal, 0, 128);
        xm_task_handle[i] = osThreadCreate(osThread(xm_task_name), xm_motor_instance[i]);
    }
}
