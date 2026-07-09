# message_center

<p align='right'>neozng1@hnu.edu.cn</p>

> v0.2 更新：每个订阅者底层的手写循环队列已替换为 **FreeRTOS 原生队列（`xQueue`）**。这消除了原实现中的数据竞争（撕裂读、`temp_size` 读改写竞争、`static` 迭代器跨任务串扰等），使得 message_center 在任务之间以及任务与中断之间都是**线程安全**的。对外的四个原有接口签名与行为保持不变，应用层无需改动；并新增了两个 **ISR 上下文安全**的收发接口。

> TODO:
>
> 支持自定义队列长度：当前基于 `xQueueOverwrite` 实现，仅支持 `QUEUE_SIZE == 1`（保留最新一条消息的 mailbox 语义）。若要支持更长的队列，需将 `PubPushMessage` 改为 `xQueueSend` 并自定义队满策略（见下文“可修改的宏”）。

## 总览和封装说明

**重要定义：**

- 发布者：发布消息的对象。发布者会将自己的消息推送给所有订阅了某个特定**话题**的订阅者。
- 订阅者：获取消息的对象。订阅者在订阅了某个话题之后，可以通过接口获得该话题的消息。
- 话题（topic）：用于区分消息来源的对象。可以将一个话题看作一刊杂志，不同的发布者会将文章汇集到杂志上，而订阅者选择订阅一种杂志，然后就可以获取所有写在杂志上的文章。

Message Center不同应用间进行消息传递的中介，它的存在可以在相当大的程度上解耦不同的app，使得不同的应用之间**不存在包含关系**，让代码的自由度更大，将不同模块之间的关系降为**松耦合**。在以往，如果一个.c文件中的数据需要被其他任务/源文件共享，那么其他模块应该要包含前者的头文件，且头文件中应当存在获取该模块数据的接口（即函数，一般是返回数据指针或直接返回数据，**强烈建议不要使用全局变量**）；但现在，不同的应用之间完全隔离，他们不需要了解彼此的存在，而是只能看见一个**消息中心**以及一些**话题**。

需要被共享的消息，将会被**发布者**（publisher）发送到消息中心；要获取消息，则由**订阅者**（subscriber）从消息中心根据订阅的话题获取。在这之前，发布者要在消息中心完成**注册**，将自己要发布的消息类型和话题名称提交到消息中心；订阅者同样要先在消息中心完成订阅，将自己要接收的消息类型和话题名称提交到订阅中心。消息中心会根据**话题名称**，把订阅者绑定到发布相同名称的发布者上。

> 数据结构上采用**链表 + FreeRTOS 队列**的方式：C 没有哈希表，因此让发布者保存所有订阅者的地址（实际上只保存首地址，然后通过链表访问所有订阅者）；每个订阅者持有一个 FreeRTOS 队列句柄作为消息缓冲，队列内部自带临界区，因此收发操作本身是原子的、线程安全的。

Message Center 对外提供了六个接口（四个原有 + 两个 ISR 版本），所有原本要进行信息交互的应用都应该包含`message_center.h`，并在初始化的时候进行注册。

> **注册时机**：所有 `SubRegister`/`PubRegister` 都应在 `osKernelStart()` 之前的各 `*Init()` 中完成。注册过程会 `malloc` 链表结点并 `xQueueCreate`，运行期不应再注册；运行期只调用收发接口。

## 代码结构

.h 文件中包含了外部接口和类型定义，.c中包含了各个接口的具体实现。

## 外部接口

**在代码实现上，话题名实际上就是通过一个字符串体现的。**

```c
// 注册（任务/初始化上下文）
Subscriber_t* SubRegister(char* name, uint8_t data_len);
Publisher_t*  PubRegister(char* name, uint8_t data_len);

// 任务上下文收发
uint8_t SubGetMessage(Subscriber_t* sub, void* data_ptr);
uint8_t PubPushMessage(Publisher_t* pub, void* data_ptr);

// 中断上下文收发（ISR 安全）
uint8_t PubPushMessageFromISR(Publisher_t* pub, void* data_ptr, BaseType_t* higher_priority_task_woken);
uint8_t SubGetMessageFromISR(Subscriber_t* sub, void* data_ptr, BaseType_t* higher_priority_task_woken);
```

### 订阅者

订阅者应该保存一个订阅者类型的指针`Subscriber_t*`，在初始化的时候调用`SubRegister()`并传入要订阅的话题名和该话题对应消息的长度，可以直接输入字符串，示例如下，将从event_name订阅float数据：

```c
Subscriber_t* my_sub;
my_sub=SubRegister("event_name",sizeof(float));
```

订阅完毕后，在应用中通过`SubGetMessage()`获取消息，调用时传入订阅时获得的指针，以及要存放数据的指针。在使用的时候，建议使用强制类型转换将`data_ptr` cast成void*类型（好习惯）。

如果消息队列中有消息，返回值为1；否则，返回值为0，说明没有新的消息可用。

### 发布者

发布者应该保存一个发布者类型的指针，在初始化的时候传入要发布的话题名和该话题对应的消息长度。

完成注册后，通过`PubPushMessage()`发布新的消息。所有订阅了该话题的订阅者都会收到新的消息推送。

### 中断（ISR）上下文

如果需要在中断服务函数中收发消息（例如在 CAN/串口接收回调里直接把最新数据发布出去），**不能**调用 `PubPushMessage()`/`SubGetMessage()`，必须使用对应的 `*FromISR` 版本。它们额外接收一个 `BaseType_t*` 参数：调用前由调用者初始化为 `pdFALSE`，若本次操作唤醒了更高优先级的任务则会被置为 `pdTRUE`，中断返回前据此触发一次调度。

```c
void SomeRxCallback(void)
{
    BaseType_t woken = pdFALSE;          // 必须先初始化为 pdFALSE
    PubPushMessageFromISR(my_pub, &data, &woken);
    portYIELD_FROM_ISR(woken);           // 若有更高优先级任务被唤醒则立即切换
}
```

> 订阅者链表在注册阶段（调度器启动前）建立、运行期不再修改，因此在中断中遍历订阅者是安全的。同一个话题可以在任务里用 `PubPushMessage()` 发布、在中断里用 `PubPushMessageFromISR()` 发布，两者都安全，只要各自使用正确的版本。

### 可修改的宏

```c
#define MAX_TOPIC_NAME_LEN  32  //最大的话题名长度,每个话题都由字符串来命名
#define QUEUE_SIZE 1            //每个订阅者的消息队列长度
```

修改第一个可以扩大话题名长度。第二个是每个订阅者队列的长度：当前实现基于 `xQueueOverwrite`（只保留最新一条消息的 mailbox 语义），因此 `QUEUE_SIZE` **固定为 1**——`message_center.c` 中有 `#if QUEUE_SIZE != 1 #error ...` 编译期断言防止误改。若确实需要更长的队列，需把 `PubPushMessage()` 从 `xQueueOverwrite` 改为 `xQueueSend`，并显式定义队满策略（丢弃最新，或先 `xQueueReceive` 丢弃最老再 `xQueueSend`），改完后再移除该断言。

## 私有函数和定义

```c
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

static void CheckName(char* name)
{
    if(strnlen(name,MAX_TOPIC_NAME_LEN+1)>=MAX_TOPIC_NAME_LEN)
    {
        LOGERROR("EVENT NAME TOO LONG:%s", name);
        while (1); // 进入这里说明话题名超出长度限制
    }
}
```

`message_center`内部保存了指向第一个发布者的指针，可以看作整个消息中心的抽象。通过这个变量，可以访问所有发布者和订阅者。它将会在各个函数中作为dumb_head（哑结点）以简化逻辑，这样不需要对链表头进行特殊处理。

`CheckName()`在发布者/订阅者注册的时候被调用，用于检查话题名是否超过长度限制。超长后会进入死循环，方便开发者检查。`CheckLen()`则检查同名话题的消息长度是否一致。注册时 `malloc`/`xQueueCreate` 失败也会打印日志并进入死循环——这些都属于开发期就应暴露的配置错误。

> 订阅者结点中不再保存手写的循环缓冲和 `front/back/temp_size` 索引，而是持有一个 FreeRTOS 队列句柄 `QueueHandle_t queue`。该队列在 `SubRegister()` 中通过 `xQueueCreate(QUEUE_SIZE, data_len)` 创建，长度为 `QUEUE_SIZE`（=1），单个元素大小为该话题的 `data_len` 字节。

> 四个外部接口的实现都有详细的注释，有兴趣的同学可以自行阅读。下方也提供了流程图。

## 注册、发布、获取消息流程

包含一个结构图和四个流程图。

### Message Center的结构![image-20221201150945052](../../.assets/image-20221201150945052.png)

<center>建议打开原图查看</center>

**多个publisher可以绑定同一个话题，往该话题推送消息。但一个subscriber只能订阅一个话题，如果应用需要订阅多个话题，则要创建对应数量的订阅者。**

> 对于电控程序目前的情况，不存在多个publisher向同一个话题推送消息的情况。

**对于相同话题，其消息长度必须相同**。发布者和订阅者在注册时都会传入消息长度，用`sizof(your_data)`获取。应当保证不同的模块在进行交互式，使用相同的数据长度。

### 发布者和订阅者注册的流程

- **发布者：**

  遍历发布者的话题结点，如果发现相同的话题，直接返回指针即可；遍历完成后发现尚未创建则创建新的话题。

<img src="../../.assets/image-20221201152530558.png" alt="image-20221201152530558" style="zoom: 80%;" />

- **订阅者：**

  需要注意，由于不同应用/模块的初始化顺序不同，可能出现订阅者先于发布者订阅某一消息的情况，所以要进行发布者链表的遍历，判断是否已经存在相同话题名的发布者，不存在则要先创建发布者结点再将新建订阅者结点并挂载到前者上。

<img src="../../.assets/image-20221201152904044.png" alt="image-20221201152904044" style="zoom:80%;" />

### 推送/获取消息的流程

- **底层由 FreeRTOS 队列实现**

  收发不再需要手动维护 `front/back/temp_size` 索引，这些都交给 FreeRTOS 队列内部处理，队列自带临界区保护，因此天然线程安全。`QUEUE_SIZE == 1` 时使用的 `xQueueOverwrite` 具有 mailbox 语义：无论队列是否已满都写入成功，始终只保留最新的一条消息（相当于旧实现中“队满则丢弃最老再写入”的退化情形）。

- **发布者推送消息到指定话题（`PubPushMessage` / `PubPushMessageFromISR`）**

  通过发布者指针遍历订阅了该话题的所有订阅者，对每个订阅者的队列各调用一次 `xQueueOverwrite`（ISR 版本为 `xQueueOverwriteFromISR`）。这样实现了“一次发布、所有订阅者各得一份”的广播（fan-out）语义——这也是不能简单地“一个话题共用一条队列”的原因：FreeRTOS 队列是点对点的，谁先取走消息谁就消费掉了，不会复制给其他读者。

- **订阅者获取消息（`SubGetMessage` / `SubGetMessageFromISR`）**

  对订阅者自己的队列调用 `xQueueReceive`，超时参数为 0（非阻塞）。取到数据返回 1；队列为空返回 0，此时 `data_ptr` 指向的内容保持不变（调用方通常沿用上一次的数据）。

## 示例代码

```c
typedef struct 
{
    float a;
    uint8_t b;
    uint32_t c;
}good;

good g1;
good g2;
good pub_data={.a=1,.b=2,.c=3};
// 一个发布者,两个订阅者
Subscriber_t* s=SubRegister("test",sizeof(good));
Subscriber_t* ss=SubRegister("test",sizeof(good));
Publisher_t* p=PubRegister("test",sizeof(good));
// 推送消息
PubPushMessage(p,&pub_data);
pub_data.a=4;
pub_data.b=5;
pub_data.c=6;
// 推送新消息
PubPushMessage(p,&pub_data);
volatile uint8_t d= 0; // 确定收到的消息是否有效,可以根据d的值进一步处理
d=SubGetMessage(s,&g1);  // d==1, g1=={4,5,6}
d=SubGetMessage(s,&g1);  // d==0, 队列已空, g1 保持不变
d=SubGetMessage(s,&g1);  // d==0
d=SubGetMessage(ss,&g2); // d==1, g2=={4,5,6}(每个订阅者有独立队列)
```

> 注意 `QUEUE_SIZE == 1` 的 mailbox 语义：上面连续两次 `PubPushMessage` 后，队列里只剩最新的一条 `{a=4,b=5,c=6}`（第二次推送覆盖了第一次）。因此每个订阅者第一次 `SubGetMessage` 取到 `{4,5,6}` 且返回 1，之后队列为空、返回 0，`g1`/`g2` 保持不变。这一行为与旧的手写实现完全一致，仅底层存储换成了 FreeRTOS 队列。
