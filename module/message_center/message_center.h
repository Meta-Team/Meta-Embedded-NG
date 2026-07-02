/**
 * @file message_center.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 应用间通信的伪pubsub机制,底层由FreeRTOS队列实现,任务间/ISR均线程安全
 * @version 0.2
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 * @note v0.2: 每个订阅者的手写循环队列被替换为FreeRTOS原生队列(xQueue),
 *       消除了原实现中的数据竞争,并新增了ISR上下文安全的收发接口。
 *       对外的四个原有接口(SubRegister/PubRegister/SubGetMessage/PubPushMessage)
 *       签名与行为保持不变,应用层无需改动。
 */

#ifndef PUBSUB_H
#define PUBSUB_H

#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"

#define MAX_TOPIC_NAME_LEN 32 // 最大的话题名长度,每个话题都有字符串来命名
#define MAX_TOPIC_COUNT 12    // 最多支持的话题数量
#define QUEUE_SIZE 1          // 每个订阅者只保留最新的一条消息(mailbox语义)

typedef struct mqt
{
    /* 底层FreeRTOS队列句柄,长度为QUEUE_SIZE,单个元素大小为data_len字节 */
    QueueHandle_t queue;
    uint8_t data_len; // 该订阅者接收的消息长度

    /* 指向下一个订阅了相同话题的订阅者的指针 */
    struct mqt *next_subs_queue; // 使得发布者可以通过链表访问所有订阅了相同话题的订阅者
} Subscriber_t;

/**
 * @brief 发布者类型.每个发布者拥有发布者实例,并且可以通过链表访问所有订阅了自己发布的话题的订阅者
 *
 */
typedef struct ent
{
    /* 话题名称 */
    char topic_name[MAX_TOPIC_NAME_LEN + 1]; // 1个字节用于存放字符串结束符 '\0'
    uint8_t data_len;                        // 该话题的数据长度
    /* 指向第一个订阅了该话题的订阅者,通过链表访问所有订阅者 */
    Subscriber_t *first_subs;
    /* 指向下一个Publisher的指针 */
    struct ent *next_topic_node;
    uint8_t pub_registered_flag; // 用于标记该发布者是否已经注册
} Publisher_t;

/**
 * @brief 订阅name的话题消息
 *
 * @param name 话题名称
 * @param data_len 消息长度,通过sizeof()获取
 * @return Subscriber_t* 返回订阅者实例
 */
Subscriber_t *SubRegister(char *name, uint8_t data_len);

/**
 * @brief 注册成为消息发布者
 *
 * @param name 发布者发布的话题名称(话题)
 * @return Publisher_t* 返回发布者实例
 */
Publisher_t *PubRegister(char *name, uint8_t data_len);

/**
 * @brief 获取消息(任务上下文调用)
 *
 * @param sub 订阅者实例指针
 * @param data_ptr 数据指针,接收的消息将会放到此处
 * @return uint8_t 返回值为0说明没有新的消息(消息队列为空),为1说明获取到了新的消息
 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);

/**
 * @brief 发布者给所有订阅了话题的订阅者推送消息(任务上下文调用)
 *
 * @param pub 发布者实例指针
 * @param data_ptr 指向要发布的数据的指针
 * @return uint8_t 恒为1(推送成功)
 */
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr);

/**
 * @brief 发布者给所有订阅者推送消息(中断上下文调用,ISR安全版本)
 *
 * @param pub 发布者实例指针
 * @param data_ptr 指向要发布的数据的指针
 * @param higher_priority_task_woken 调用前必须由调用者初始化为pdFALSE;若本次推送
 *        唤醒了更高优先级的任务,会被置为pdTRUE。中断返回前应据此调用
 *        portYIELD_FROM_ISR(*higher_priority_task_woken) 触发一次调度
 * @return uint8_t 恒为1(推送成功)
 *
 * @code
 *   BaseType_t woken = pdFALSE;
 *   PubPushMessageFromISR(pub, &data, &woken);
 *   portYIELD_FROM_ISR(woken);
 * @endcode
 */
uint8_t PubPushMessageFromISR(Publisher_t *pub, void *data_ptr, BaseType_t *higher_priority_task_woken);

/**
 * @brief 获取消息(中断上下文调用,ISR安全版本)
 *
 * @param sub 订阅者实例指针
 * @param data_ptr 数据指针,接收的消息将会放到此处
 * @param higher_priority_task_woken 调用前必须由调用者初始化为pdFALSE;语义同上,
 *        中断返回前应调用 portYIELD_FROM_ISR(*higher_priority_task_woken)
 * @return uint8_t 返回值为0说明没有新的消息(消息队列为空),为1说明获取到了新的消息
 */
uint8_t SubGetMessageFromISR(Subscriber_t *sub, void *data_ptr, BaseType_t *higher_priority_task_woken);

#endif // !PUBSUB_H
