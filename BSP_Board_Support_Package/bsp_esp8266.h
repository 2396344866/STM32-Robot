#ifndef __BSP_ESP8266_H
#define __BSP_ESP8266_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ESP8266_RX_MAX 512
#define ESP8266_BUF_N 4  // AT 模式资源池块数（透传模式不使用该池，但为支持运行时切换仍分配）

// ===== 工作模式（编译期宏，决定默认模式；亦可运行时 BSP_ESP8266_SwitchMode 切换）=====
// 0 = AT 指令模式（非透传）：收到 +IPD/+MQTT 固件封装帧，帧边界 100% 可靠
//     → Normal + 双队列(FreeQueue/DataQueue) 硬背压（FreeQueue 空即丢包，绝不践踏）。
//     适合：低功耗间歇上传（请求-响应，帧边界清晰，CPU 处理完即可休眠）。
// 1 = 透传模式（Transparent）：纯裸流无前缀
//     → Circular + 环形拷贝，DMA 持续运行不暂停（适合满负荷高速上行）。
// 切换方式：修改此宏重新编译；或运行时调用 BSP_ESP8266_SwitchMode()（两套缓冲同时存在，付出少量 RAM）。
#define ESP8266_TRANSPARENT_MODE  0

// ---- 透传模式（环形）所需缓冲 ----
extern uint8_t g_esp8266_rx_buf[ESP8266_RX_MAX];
extern volatile uint16_t g_rx_read_idx;  // 环形读指针（仅任务上下文更新）

// ---- AT 模式（双队列）所需缓冲 ----
typedef struct {
    uint8_t data[ESP8266_RX_MAX];
    uint16_t len;  // 本块实际接收字节数
} esp8266_buf_t;
extern esp8266_buf_t g_esp8266_pool[ESP8266_BUF_N];
extern QueueHandle_t xEspFreeQueue;   // 空闲池：初始全放 N 块
extern QueueHandle_t xEspDataQueue;   // 数据池：待解析块，初始为空
extern uint32_t g_esp_rx_overflow_cnt; // FreeQueue 为空时主动丢弃计数（硬背压生效记录）
extern volatile uint8_t g_esp_dma_suspended; // DMA 暂停标志：FreeQueue 空时置位，任务归还后清除

// 当前生效模式：0=AT(Normal+双队列)，1=透传(Circular+环形)。默认由 ESP8266_TRANSPARENT_MODE 设定，可运行时切换。
extern volatile uint8_t g_esp_mode;

// 初始化；net_task_handle 为被唤醒的网络任务句柄（ISR 用 xTaskNotify 替代二值信号量）
void BSP_ESP8266_Init(uint32_t baudrate, TaskHandle_t net_task_handle);
void BSP_ESP8266_SendString(const char* str);

// 透传模式：二进制安全发送（不依赖字符串结尾 '\0'，适合发裸 MQTT 报文）。
// 注意：透传模式下所有字节直接进 TCP socket，因而该函数仅在 g_esp_mode=1（已进入透传）后使用。
void BSP_ESP8266_SendRaw(const uint8_t* data, uint16_t len);

// 低功耗：ESP8266 modem sleep 开关。enable=1 发 AT+SLEEP=1（自动休眠，有数据自动唤醒）；
// enable=0 发 AT+SLEEP=0 退出。仅 AT 模式（g_esp_mode=0）下有效；透传模式无法发 AT 命令，调用将被忽略。
void BSP_ESP8266_SetModemSleep(uint8_t enable);

// 透传模式：从 DMA 环形缓冲拷贝可用字节到 out_buf（回绕安全），更新读指针，返回拷贝字节数
uint16_t BSP_ESP8266_RxDrain(char* out_buf, uint16_t out_max);

// AT 模式：任务上下文恢复 DMA（背压解除后调用，从空闲池取一块重启）
void BSP_ESP8266_ResumeDMA(void);

// 运行时切换模式：transparent=1 透传(Circular)，=0 AT(Normal+双队列)。返回 1=已切换，0=无需切换。
// 用途：满负荷时切透传高速上行；进入低功耗时切回 AT 间歇上传（帧边界清晰，便于休眠）。
int  BSP_ESP8266_SwitchMode(uint8_t transparent);

#endif
