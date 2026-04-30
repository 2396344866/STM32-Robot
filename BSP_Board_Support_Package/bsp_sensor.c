#include "bsp_sensor.h"
#include "stm32f10x.h"
#include "hal_gpio.h"
#include "hal_delay.h"
#include "hal_adc_dma.h"
#include "hal_tim_ic.h"
#include "dev_mq2.h"
#include "dev_hcsr04.h"
#include "dev_dht11.h"

// 内部句柄实例
static MQ2_Handle_t hMq2;
static HCSR04_Handle_t hHcsr04;
static DHT11_Handle_t hDht11;

// 内部 DMA 缓冲区
static uint16_t adc_dma_buffer[4];

// --- MQ2 回调函数 ---
static uint16_t BSP_MQ2_ReadADC(void) {
    return adc_dma_buffer[0];
}

// --- 读取 Rank 2 (PA5) ---
float BSP_Sensor_GetLight(void) {
    uint16_t adc_val = adc_dma_buffer[1];
    
    // 硬件标定与勒克斯映射参数
    const float ADC_DARK = 4095.0f;   // 全黑环境 ADC 值
    const float ADC_BRIGHT = 2000.0f; // 强光环境 ADC 值
    const float MAX_LUX = 1000.0f;    // 设定的最大参考照度 (Lux)
    
    float lux = 0.0f;
    
    // 1. 低于暗阈值，输出 0 Lux
    if (adc_val >= ADC_DARK) {
        lux = 0.0f;
    } 
    // 2. 高于亮阈值，钳位最大 Lux
    else if (adc_val <= ADC_BRIGHT) {
        lux = MAX_LUX;
    } 
    // 3. 线性插值映射
    else {
        lux = (ADC_DARK - (float)adc_val) / (ADC_DARK - ADC_BRIGHT) * MAX_LUX;
    }
    
    return lux;
}

// --- HCSR04 回调函数 ---
static void BSP_HCSR04_Trig(uint8_t state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_Pin_8, state ? HAL_PIN_SET : HAL_PIN_RESET);
}

// --- DHT11 回调函数 ---
static void BSP_DHT11_SetOut(void) {
    HAL_GPIO_Init(GPIOA, GPIO_Pin_15, HAL_GPIO_MODE_OUTPUT_PP);
}
static void BSP_DHT11_SetIn(void) {
    HAL_GPIO_Init(GPIOA, GPIO_Pin_15, HAL_GPIO_MODE_INPUT_PU); // 上拉输入
}
static void BSP_DHT11_Write(uint8_t state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_Pin_15, state ? HAL_PIN_SET : HAL_PIN_RESET);
}
static uint8_t BSP_DHT11_Read(void) {
    return (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_Pin_15);
}

// --- 系统初始化 ---
void BSP_Sensors_Init(void) {
    // 1. 释放 PA15 (针对 DHT11)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    // 2. 初始化 MQ2 底层硬件与句柄
    ADC_DMA_Handle_t adc_handle;
    
		// 定义通道顺序: 0=烟雾, 1=光照
    static uint8_t adc_channels[2] = {ADC_Channel_4, ADC_Channel_5};
		
		
		adc_handle.ADCx = ADC1;
    adc_handle.GPIOx = GPIOA;
    // 物理引脚作或运算传入
    adc_handle.GPIO_Pins = GPIO_Pin_4 | GPIO_Pin_5;
    adc_handle.ADC_Channels = adc_channels;
    adc_handle.ChannelCount = 2;
    adc_handle.DMABuffer = adc_dma_buffer;
    
    HAL_ADC_DMA_Init(&adc_handle);
    
    // 初始化 MQ2 句柄
    Dev_MQ2_Init(&hMq2, BSP_MQ2_ReadADC);
		
		
    // 3. 初始化 HCSR04 底层硬件与句柄
    HAL_GPIO_Init(GPIOA, GPIO_Pin_8, HAL_GPIO_MODE_OUTPUT_PP);
    HAL_TIM1_CH4_IC_Init(GPIOA, GPIO_Pin_11);
    Dev_HCSR04_Init(&hHcsr04, BSP_HCSR04_Trig, HAL_Delay_us, HAL_TIM1_CH4_GetPulseWidth);

    // 4. 初始化 DHT11 句柄
    hDht11.SetPinOut = BSP_DHT11_SetOut;
    hDht11.SetPinIn = BSP_DHT11_SetIn;
    hDht11.WritePin = BSP_DHT11_Write;
    hDht11.ReadPin = BSP_DHT11_Read;
    hDht11.DelayUs = HAL_Delay_us;
    hDht11.DelayMs = HAL_Delay_ms;
    Dev_DHT11_Init(&hDht11);
}

// --- 数据读取接口 ---
float BSP_Sensor_GetDistance(void) {
    return Dev_HCSR04_GetDistance(&hHcsr04);
}

float BSP_Sensor_GetSmoke(void) {
    return Dev_MQ2_GetPPM(&hMq2);
}

uint8_t BSP_Sensor_ReadDHT11(uint8_t *temp, uint8_t *humi) {
    return Dev_DHT11_Read(&hDht11, temp, humi);
}
