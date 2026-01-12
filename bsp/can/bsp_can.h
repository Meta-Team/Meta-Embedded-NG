#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "fdcan.h"

// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_FDCAN_FILTER_CNT 28     // FDCAN每个实例最多支持28个标准ID过滤器
#define DEVICE_CAN_CNT 3           // 根据板子设定,H723有FDCAN1,FDCAN2,FDCAN3,因此为3

/* CAN ID类型枚举,用于区分标准ID(11位)和扩展ID(29位) */
typedef enum
{
    CAN_ID_STD = 0,  // 标准ID,11位
    CAN_ID_EXT = 1   // 扩展ID,29位
} CAN_ID_Type_e;

/* can instance typedef, every module registered to CAN should have this variable */
#pragma pack(1)
typedef struct _
{
    FDCAN_HandleTypeDef *can_handle; // can句柄
    FDCAN_TxHeaderTypeDef txconf;    // FDCAN报文发送配置
    uint32_t tx_id;                  // 发送id
    uint8_t tx_buff[8];              // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];              // 接收缓存,最大消息长度为8
    uint32_t rx_id;                  // 接收id(用于匹配)
    uint32_t rx_id_mask;             // 接收id掩码,用于灵活匹配(如CyberGear等协议)
    uint32_t last_rx_identifier;     // 最后接收到的完整ID,用于解析扩展协议信息
    uint8_t rx_len;                  // 接收长度,可能为0-8
    CAN_ID_Type_e id_type;           // ID类型,标准ID或扩展ID
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
#pragma pack()

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    FDCAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    uint32_t rx_id_mask;                        // 接收id掩码,0表示使用默认精确匹配(STD:0x7FF, EXT:0x1FFFFFFF)
    CAN_ID_Type_e id_type;                      // ID类型,CAN_ID_STD(标准11位)或CAN_ID_EXT(扩展29位)
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief 动态修改CAN发送报文的ID,用于支持CyberGear等需要动态ID的协议
 *
 * @param _instance 要修改发送ID的can实例
 * @param tx_id     新的发送ID
 */
void CANSetTxId(CANInstance *_instance, uint32_t tx_id);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 * 
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 * 
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

#endif
