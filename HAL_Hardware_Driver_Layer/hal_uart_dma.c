#include "hal_uart_dma.h"

void HAL_UART_DMA_Rx_Init(UART_Handle_t* hUart, uint8_t* rx_buffer, uint16_t buffer_size)
{
    DMA_InitTypeDef DMA_InitStructure;
    DMA_Channel_TypeDef* dma_channel = NULL;

    // 1. 使能 DMA1 时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 2. 匹配 DMA 通道
    if (hUart->Instance == USART1) {
        dma_channel = DMA1_Channel5;
    } else if (hUart->Instance == USART2) {
        dma_channel = DMA1_Channel6;
    } else {
        return; // STM32F103 中 USART3_RX 对应 DMA1_Ch3，此处按需扩充
    }

    // 3. 配置 DMA 接收
    DMA_DeInit(dma_channel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&hUart->Instance->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = buffer_size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(dma_channel, &DMA_InitStructure);

    // 4. 开启串口的 DMA 接收请求 与 空闲中断 (IDLE)
    USART_DMACmd(hUart->Instance, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(hUart->Instance, USART_IT_IDLE, ENABLE);
    
    // 5. 使能 DMA 通道
    DMA_Cmd(dma_channel, ENABLE);
}
