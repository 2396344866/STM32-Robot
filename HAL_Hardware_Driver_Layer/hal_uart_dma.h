#ifndef __HAL_UART_DMA_H
#define __HAL_UART_DMA_H

#include "hal_uart.h"

// 仅初始化 DMA 接收通道，不涉及具体引脚和业务队列
void HAL_UART_DMA_Rx_Init(UART_Handle_t* hUart, uint8_t* rx_buffer, uint16_t buffer_size);

#endif
