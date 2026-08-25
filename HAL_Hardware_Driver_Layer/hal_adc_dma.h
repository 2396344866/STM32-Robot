#ifndef __HAL_ADC_DMA_H
#define __HAL_ADC_DMA_H
#include "stm32f10x.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "event_groups.h"

// ADC与DMA句柄
typedef struct {
    ADC_TypeDef* ADCx;
    GPIO_TypeDef* GPIOx;
    uint16_t      GPIO_Pins;       // 支持多个引脚 (例如 GPIO_Pin_4 | GPIO_Pin_5)
    uint8_t* ADC_Channels;    // 通道数组指针 (例如 {ADC_Channel_4, ADC_Channel_5})
    uint8_t       ChannelCount;    // 总采集通道数
    uint16_t* DMABuffer;       // 指向外部数据存储区（双缓冲：大小为 2*ChannelCount）
} ADC_DMA_Handle_t;

/*
 * ADC 双缓冲 ping-pong + 全满/半满中断。
 * - 缓冲按 2*ChannelCount 分配：前半 [0..N-1] / 后半 [N..2N-1]
 * - DMA 循环模式：传输到前半完成时触发 半满(HT) 中断，
 *   传输到后半完成时触发 全满(TC) 中断；ISR 仅置事件位唤醒消费任务，
 *   数据由任务在“非活跃”半区安全拷贝，杜绝读写撕裂。
 */
void HAL_ADC_DMA_Init(ADC_DMA_Handle_t* handle);

/* 数据就绪事件组位（由 DMA ISR 置位，bsp_sensor 任务等待） */
extern EventGroupHandle_t xAdcDataEvent;
#define ADC_EVT_HALF_READY  (1u << 0)   // 前半区数据就绪
#define ADC_EVT_FULL_READY  (1u << 1)   // 后半区数据就绪

/* 从“非活跃半区”拷贝一帧采样值到 out（长度 = ChannelCount），返回 true 表示有更新 */
bool HAL_ADC_DMA_GetFrame(ADC_DMA_Handle_t* handle, uint16_t* out, uint8_t which_half);

/* DMA1 通道1 中断服务例程（半满/全满 + 传输错误） */
void DMA1_Channel1_IRQHandler(void);

#endif
