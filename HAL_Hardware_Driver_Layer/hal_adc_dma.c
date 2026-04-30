#include "hal_adc_dma.h"

void HAL_ADC_DMA_Init(ADC_DMA_Handle_t* handle) {
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

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

    // 3. 配置 DMA
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(handle->ADCx->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)handle->DMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    
    // 【核心变更】传输数量 = 通道数；内存是否递增由通道数决定
    DMA_InitStructure.DMA_BufferSize = handle->ChannelCount; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = (handle->ChannelCount > 1) ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable; 
    
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // 4. 配置 ADC
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    
    // 【核心变更】多通道必须开启扫描模式
    ADC_InitStructure.ADC_ScanConvMode = (handle->ChannelCount > 1) ? ENABLE : DISABLE; 
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = handle->ChannelCount; // 转换通道数
    ADC_Init(handle->ADCx, &ADC_InitStructure);

    // 【核心变更】循环注册所有传入的通道序列 (Rank)
    for (uint8_t i = 0; i < handle->ChannelCount; i++) {
        // i+1 即为序列 Rank（从 1 开始排队）
        ADC_RegularChannelConfig(handle->ADCx, handle->ADC_Channels[i], i + 1, ADC_SampleTime_239Cycles5);
    }

    ADC_DMACmd(handle->ADCx, ENABLE);
    ADC_Cmd(handle->ADCx, ENABLE);
    
    // 5. ADC 校准
    ADC_ResetCalibration(handle->ADCx);
    while(ADC_GetResetCalibrationStatus(handle->ADCx));
    ADC_StartCalibration(handle->ADCx);
    while(ADC_GetCalibrationStatus(handle->ADCx));
    
    // 6. 启动连续转换
    ADC_SoftwareStartConvCmd(handle->ADCx, ENABLE);
}
