#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给FDCAN添加过滤器后,FDCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note H723的FDCAN每个实例有独立的过滤器,每个实例最多支持28个标准ID过滤器
 *       所有消息都会被路由到RxFifo0
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static void CANAddFilter(CANInstance *_instance)
{
    FDCAN_FilterTypeDef fdcan_filter_conf;
    static uint8_t fdcan1_filter_idx = 0, fdcan2_filter_idx = 0; // 每个FDCAN实例有独立的过滤器索引

    fdcan_filter_conf.IdType = FDCAN_STANDARD_ID;              // 使用标准ID
    fdcan_filter_conf.FilterIndex = _instance->can_handle == &hfdcan1 ? (fdcan1_filter_idx++) : (fdcan2_filter_idx++); // 根据can_handle判断是FDCAN1还是FDCAN2,然后自增
    fdcan_filter_conf.FilterType = FDCAN_FILTER_MASK;          // 使用掩码模式
    fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 消息路由到RxFifo0
    fdcan_filter_conf.FilterID1 = _instance->rx_id;            // 过滤ID
    fdcan_filter_conf.FilterID2 = 0x7FF;                        // 掩码,0x7FF表示精确匹配11位标准ID

    HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf);
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动FDCAN1和FDCAN2,开启FDCAN1和FDCAN2的RxFifo0接收通知
 *
 */
static void CANServiceInit()
{
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit(); // 第一次注册,先进行硬件初始化
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
                LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }
    
    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
    instance->txconf.Identifier = config->tx_id;          // 发送id
    instance->txconf.IdType = FDCAN_STANDARD_ID;          // 使用标准id,扩展id则使用FDCAN_EXTENDED_ID(目前没有需求)
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;      // 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8;      // 默认发送长度为8字节
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;       // 经典CAN模式,不使用位速率切换
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;        // 使用经典CAN格式
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle) == 0) // 等待TxFifo空闲
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            LOGWARNING("[bsp_can] FDCAN TxFifo full! failed to add msg to fifo. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;
    // 发送消息到TxFifo
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
    {
        LOGWARNING("[bsp_can] FDCAN bus BUSY! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
            LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
    
    // 将字节长度转换为FDCAN的DLC代码
    uint32_t dlc_codes[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };
    _instance->txconf.DataLength = dlc_codes[length];
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief 此函数会被下面的函数调用,用于处理RxFifo0接收中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan FDCAN句柄
 * @param fifox FIFO索引(FDCAN使用RxFifo0)
 */
static void CANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    FDCAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        if (HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) != HAL_OK) // 从FIFO中获取数据
            continue;
        
        uint32_t rx_id = (rxconf.IdType == FDCAN_STANDARD_ID) ? rxconf.Identifier : (rxconf.Identifier & 0x7FF);
        
        for (size_t i = 0; i < idx; ++i)
        { // 两者相等说明这是要找的实例
            if (_hcan == can_instance[i]->can_handle && rx_id == can_instance[i]->rx_id)
            {
                if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
                {
                    // FDCAN的DataLength是DLC代码,需要转换为实际字节数
                    can_instance[i]->rx_len = (rxconf.DataLength >> 16) & 0x0F; // 提取实际字节数
                    if (can_instance[i]->rx_len > 8) can_instance[i]->rx_len = 8;
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, can_instance[i]->rx_len); // 消息拷贝到对应实例
                    can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}

/**
 * @brief 注意,STM32H7的FDCAN每个实例有独立的FIFO
 * 下面的函数是HAL库中的回调函数,它被HAL声明为__weak,这里对其进行重载(重写)
 * 当RxFifo0有新消息时会调用此函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定FDCAN设备的消息

/**
 * @brief rx fifo callback. Once RxFifo0 receives new message, this func would be called
 *
 * @param hfdcan FDCAN handle indicate which device the message comes from
 * @param RxFifo0ITs 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
    }
}
