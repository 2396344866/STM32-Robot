#ifndef __HAL_UART_DMA_H
#define __HAL_UART_DMA_H

#include "hal_uart.h"

// 仅初始化 DMA 接收通道，不涉及具体引脚和业务队列
// dma_mode: DMA_Mode_Circular（ESP8266 环形，DMA 持续写）或 DMA_Mode_Normal（DEBUG 双队列，每帧停由 ISR 重启）
void HAL_UART_DMA_Rx_Init(UART_Handle_t* hUart, uint8_t* rx_buffer, uint16_t buffer_size, uint32_t dma_mode);

#endif
