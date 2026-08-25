#include "hal_adc_dma.h"
#include "hal_rcc.h"
#include "hal_hw_cfg.h"  // 魔法数字宏 + assert_param

/* DMA 接收事件组（ISR -> 消费任务 同步原语） */
EventGroupHandle_t xAdcDataEvent = NULL;

void HAL_ADC_DMA_Init(ADC_DMA_Handle_t* handle) {
    assert_param(handle != NULL);
    assert_param(handle->DMABuffer != NULL);
    assert_param(handle->ADC_Channels != NULL);
    assert_param(handle->ChannelCount > 0 && handle->ChannelCount * 2 <= HAL_HW_CFG_ADC_DMA_BUF_WORDS);
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    if (xAdcDataEvent == NULL) {
        xAdcDataEvent = xEventGroupCreate();
    }

    // 1. 开启时钟
    if (handle->ADCx == ADC1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    }
    if (handle->GPIOx == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置 GPIO
    GPIO_InitStructure.GPIO_Pin = handle->GPIO_Pins;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(handle->GPIOx, &GPIO_InitStructure);

    // 3. 配置 DMA —— 双缓冲：总大小 2*ChannelCount，循环模式
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(handle->ADCx->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)handle->DMABuffer;     // 指向双缓冲首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = (uint16_t)(handle->ChannelCount * 2); // 双缓冲长度(2*ChannelCount, 上限见 HAL_HW_CFG_ADC_DMA_BUF_WORDS)
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // 4. 开启 半满(HT)/全满(TC)/传输错误(TE) 中断 —— 工业标准 ping-pong 做法
    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    // 落在 FreeRTOS 管控区间(>=10)，保证 ISR 内 xEventGroupSetBitsFromISR 合法
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 配置 ADC
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = (handle->ChannelCount > 1) ? ENABLE : DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = handle->ChannelCount;
    ADC_Init(handle->ADCx, &ADC_InitStructure);

    for (uint8_t i = 0; i < handle->ChannelCount; i++) {
        ADC_RegularChannelConfig(handle->ADCx, handle->ADC_Channels[i], i + 1, HAL_HW_CFG_ADC_SAMPLE_CYCLES);
    }

    ADC_DMACmd(handle->ADCx, ENABLE);
    ADC_Cmd(handle->ADCx, ENABLE);

    // 6. ADC 校准
    ADC_ResetCalibration(handle->ADCx);
    while(ADC_GetResetCalibrationStatus(handle->ADCx));
    ADC_StartCalibration(handle->ADCx);
    while(ADC_GetCalibrationStatus(handle->ADCx));

    // 7. 启动连续转换
    ADC_SoftwareStartConvCmd(handle->ADCx, ENABLE);
}

/*
 * 从“非活跃半区”取一帧。which_half:
 *   ADC_EVT_HALF_READY -> 当前应读前半区(HT 触发，后半区正被 DMA 写)
 *   ADC_EVT_FULL_READY -> 当前应读后半区(TC 触发，前半区正被 DMA 写)
 * 因 ISR 仅在半区切换时置位，消费任务在另一半区拷贝，天然避免读写撕裂。
 */
bool HAL_ADC_DMA_GetFrame(ADC_DMA_Handle_t* handle, uint16_t* out, uint8_t which_half) {
    assert_param(handle != NULL);
    assert_param(out != NULL);
    if (!handle || !out) return false;
    const uint8_t n = handle->ChannelCount;
    const uint16_t* src = (which_half == ADC_EVT_FULL_READY)
                           ? &handle->DMABuffer[n]   // 后半区
                           : &handle->DMABuffer[0];  // 前半区
    for (uint8_t i = 0; i < n; i++) out[i] = src[i];
    return true;
}

/* DMA1 通道1 中断：半满/全满触发，仅置事件位唤醒消费任务（零数据拷贝入队） */
void DMA1_Channel1_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (DMA_GetITStatus(DMA1_IT_HT1) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        xEventGroupSetBitsFromISR(xAdcDataEvent, ADC_EVT_HALF_READY, &xHigherPriorityTaskWoken);
    }
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        xEventGroupSetBitsFromISR(xAdcDataEvent, ADC_EVT_FULL_READY, &xHigherPriorityTaskWoken);
    }
    if (DMA_GetITStatus(DMA1_IT_TE1) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TE1);
        // 传输错误：置双位通知，供任务进入安全模式/重初始化
        xEventGroupSetBitsFromISR(xAdcDataEvent,
                                  ADC_EVT_HALF_READY | ADC_EVT_FULL_READY,
                                  &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
