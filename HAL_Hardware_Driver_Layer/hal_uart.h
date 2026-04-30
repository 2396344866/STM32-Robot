#ifndef __HAL_UART_H
#define __HAL_UART_H

#include "stm32f10x.h"
#include <stdio.h> // for fputc

// 定义串口句柄
typedef struct {
    USART_TypeDef* Instance;      // 如 USART1, USART2
    uint32_t       BaudRate;      // 波特率
    // 可扩展其他参数 (Parity, StopBits 等，目前简化处理)
} UART_Handle_t;

// API 接口
void HAL_UART_Init(UART_Handle_t* hUart);
void HAL_UART_SendByte(UART_Handle_t* hUart, uint8_t byte);
void HAL_UART_SendString(UART_Handle_t* hUart, const char* str);
uint8_t HAL_UART_ReceiveByte(UART_Handle_t* hUart);

// 中断相关辅助 (可选，根据需要扩展)
void HAL_UART_EnableIRQ(UART_Handle_t* hUart, uint8_t Priority);
void MPL_LOGI(const char* fmt, ...);
void MPL_LOGE(const char* fmt, ...);
#endif
