#include "message_center.h"
#include "stdlib.h"
#include "string.h"
#include "bsp_log.h"

/* 当前实现基于 xQueueOverwrite,该接口要求队列长度为1(mailbox语义)。
 * 若将来需要更长的队列,请把 PubPushMessage 改为 xQueueSend 并自定义队满策略
 * (例如"丢弃最老"需先 xQueueReceive 再 xQueueSend),届时可移除此断言。 */
#if QUEUE_SIZE != 1
#error "message_center 当前实现仅支持 QUEUE_SIZE == 1"
#endif

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

static void CheckName(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        LOGERROR("EVENT NAME TOO LONG:%s", name);
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        LOGERROR("EVENT LEN NOT SAME:%d,%d", len1, len2);
        while (1)
            ; // 进入这里说明相同话题的消息长度却不同
    }
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CheckName(name);
    Publisher_t *node = &message_center;
    while (node->next_topic_node) // message_center会直接跳过,不需要特殊处理,这被称作dumb_head(编程技巧)
    {
        node = node->next_topic_node;            // 切换到下一个发布者(话题)结点
        if (strcmp(node->topic_name, name) == 0) // 如果已经注册了相同的话题,直接返回结点指针
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            return node;
        }
    } // 遍历完发现尚未创建name对应的话题
    // 在链表尾部创建新的话题并初始化
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    if (node->next_topic_node == NULL)
    {
        LOGERROR("PUB MALLOC FAIL:%s", name);
        while (1)
            ; // 内存不足,无法创建新话题
    }
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    Publisher_t *pub = PubRegister(name, data_len); // 查找或创建该话题的发布者
    // 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    if (ret == NULL)
    {
        LOGERROR("SUB MALLOC FAIL:%s", name);
        while (1)
            ; // 内存不足,无法创建新订阅者
    }
    memset(ret, 0, sizeof(Subscriber_t));
    // 对新建的Subscriber进行初始化
    ret->data_len = data_len; // 设定数据长度
    // 用FreeRTOS队列作为底层存储,长度QUEUE_SIZE(=1),元素大小为data_len字节;
    // 队列内部自带临界区,任务间/ISR读写均线程安全
    ret->queue = xQueueCreate(QUEUE_SIZE, data_len);
    if (ret->queue == NULL)
    {
        LOGERROR("SUB QUEUE CREATE FAIL:%s", name);
        while (1)
            ; // FreeRTOS堆不足,无法创建队列
    }
    // 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者(详见文档)
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        return ret;
    }
    // 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
    Subscriber_t *sub = pub->first_subs; // 作为iterator
    while (sub->next_subs_queue)         // 遍历订阅了该话题的订阅者链表
    {
        sub = sub->next_subs_queue; // 移动到下一个订阅者,遇到空指针停下,说明到了链表尾部
    }
    sub->next_subs_queue = ret; // 把刚刚创建的订阅者接上
    return ret;
}

/* 如果队列为空,会返回0;成功获取数据,返回1 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    // 非阻塞获取(超时为0):有消息则取出并返回1,队列为空返回0,data_ptr保持不变
    return (xQueueReceive(sub->queue, data_ptr, 0) == pdTRUE) ? 1 : 0;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    Subscriber_t *iter = pub->first_subs; // iter作为订阅者指针,遍历订阅该话题的所有订阅者
    // 遍历订阅了当前话题的所有订阅者,依次填入最新消息
    while (iter)
    {
        // 长度1队列的覆盖写:无论队列是否已满都会成功,始终保留最新的一条消息
        xQueueOverwrite(iter->queue, data_ptr);
        iter = iter->next_subs_queue; // 访问下一个订阅者
    }
    return 1;
}

uint8_t PubPushMessageFromISR(Publisher_t *pub, void *data_ptr, BaseType_t *higher_priority_task_woken)
{
    Subscriber_t *iter = pub->first_subs;
    // 订阅者链表在注册阶段(调度器启动前)建立,运行期不再修改,故ISR中遍历是安全的
    while (iter)
    {
        // ISR安全的覆盖写;同一个woken标志在整个循环中累积(FromISR接口只会将其置为pdTRUE)
        xQueueOverwriteFromISR(iter->queue, data_ptr, higher_priority_task_woken);
        iter = iter->next_subs_queue;
    }
    return 1;
}

uint8_t SubGetMessageFromISR(Subscriber_t *sub, void *data_ptr, BaseType_t *higher_priority_task_woken)
{
    return (xQueueReceiveFromISR(sub->queue, data_ptr, higher_priority_task_woken) == pdTRUE) ? 1 : 0;
}
