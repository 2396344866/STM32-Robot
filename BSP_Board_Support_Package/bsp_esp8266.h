#ifndef __BSP_ESP8266_H
#define __BSP_ESP8266_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define ESP8266_RX_MAX 512

extern uint8_t g_esp8266_rx_buf[ESP8266_RX_MAX];
extern SemaphoreHandle_t xNetRxSem;  // 二进制信号量：仅 ISR 唤醒网络任务，数据由消费者从环形缓冲自取（不传长度）

// 环形接收：从 DMA 环形缓冲中拷贝当前可用字节到 out_buf（自动处理回绕），更新读指针，返回拷贝字节数
uint16_t BSP_ESP8266_RxDrain(char* out_buf, uint16_t out_max);
void BSP_ESP8266_Init(uint32_t baudrate);
void BSP_ESP8266_SendString(const char* str);

#endif
