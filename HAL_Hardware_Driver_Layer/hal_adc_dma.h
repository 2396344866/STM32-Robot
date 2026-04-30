#ifndef __HAL_ADC_DMA_H
#define __HAL_ADC_DMA_H
#include "stm32f10x.h"

// ADC与DMA句柄
typedef struct {
    ADC_TypeDef* ADCx;
    GPIO_TypeDef* GPIOx;
    uint16_t      GPIO_Pins;       // 改为复数，支持传入多个引脚 (例如 GPIO_Pin_4 | GPIO_Pin_5)
    uint8_t* ADC_Channels;    // 通道数组指针 (例如 {ADC_Channel_4, ADC_Channel_5})
    uint8_t       ChannelCount;    // 总采集通道数
    uint16_t* DMABuffer;       // 指向外部数据存储区
} ADC_DMA_Handle_t;

void HAL_ADC_DMA_Init(ADC_DMA_Handle_t* handle);

#endif


