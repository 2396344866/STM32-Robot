#include "bsp_sensor.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "hal_gpio.h"
#include "hal_delay.h"
#include "hal_adc_dma.h"
#include "hal_tim_ic.h"
#include "hal_rcc.h"
#include "hal_hw_cfg.h"  // 魔法数字宏 + assert_param
#include "bsp_esp8266.h"  // 低功耗：ESP8266 modem sleep 接口
#include "dev_mq2.h"
#include "dev_hcsr04.h"
#include "dev_dht11.h"
#include "sys_config.h"
#include "semphr.h"
// 内部句柄实例
static MQ2_Handle_t hMq2;
static HCSR04_Handle_t hHcsr04;
static DHT11_Handle_t hDht11;

// 内部 DMA 双缓冲（ping-pong）：大小为 2*ChannelCount = 4（前半[0,1] / 后半[2,3]）
static uint16_t adc_dma_buffer[HAL_HW_CFG_ADC_DMA_BUF_WORDS];  // 双缓冲: 2*ChannelCount(<=2)=4
static uint16_t adc_frame_cache[2];   // 任务侧最新一帧（非活跃半区拷贝，防撕裂）
static ADC_DMA_Handle_t adc_handle_cache;  // 保存句柄供中断/同步使用

// --- MQ2 回调函数 ---
static uint16_t BSP_MQ2_ReadADC(void) {
    return adc_frame_cache[0];
}

// --- 读取 Rank 2 (PA5) ---
float BSP_Sensor_GetLight(void) {
    uint16_t adc_val = adc_frame_cache[1];
    
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
SemaphoreHandle_t xSensorDataMutex = NULL;   // 跨任务共享 g_sensor_data 的互斥保护
void BSP_Sensors_Init(void) {
    assert_param(xSensorDataMutex == NULL);  // 禁止重复初始化
    // 创建传感器数据互斥量（供 Sensor 写 / 网络读 保护 float 多字节读写）
    if (xSensorDataMutex == NULL) {
        xSensorDataMutex = xSemaphoreCreateMutex();
    }

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

    adc_handle_cache = adc_handle;   // 保存句柄供 BSP_Sensor_SyncAdcFrame 使用

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

/*
 * 从 ADC 双缓冲同步一帧到本地缓存。
 * 等待 DMA 半满/全满事件，从“非活跃半区”拷贝，避免读写撕裂。
 * which_half 由调用者按事件位决定。超时则保留上一帧（降级而非崩溃）。
 */
void BSP_Sensor_SyncAdcFrame(void) {
    if (xAdcDataEvent == NULL) return;
    EventBits_t bits = xEventGroupWaitBits(xAdcDataEvent,
                                           ADC_EVT_HALF_READY | ADC_EVT_FULL_READY,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(HAL_HW_CFG_SENSOR_SYNC_TIMEOUT_MS));
    uint8_t which = (bits & ADC_EVT_FULL_READY) ? ADC_EVT_FULL_READY : ADC_EVT_HALF_READY;
    HAL_ADC_DMA_GetFrame(&adc_handle_cache, adc_frame_cache, which);
}


// --- 低功耗管理接口 ---

// 射频休眠状态机镜像：避免每个 tickless 空闲周期重复下发 AT 命令（仅在状态翻转时下发一次）。
static uint8_t s_esp_lp_state = 0;

void BSP_Sensors_Sleep(void) {
    // 1. 外设级门控：停 ADC1 转换与 TIM1
    ADC_Cmd(ADC1, DISABLE);
    TIM_Cmd(TIM1, DISABLE);

    // 2. 总线级时钟门控（动态时钟门控）：关闭 ADC1 / TIM1 所在 APB2 外设时钟
    //    注意：GPIOA 时钟保持开启（引脚高阻配置仍需访问 GPIO 寄存器）。
    HAL_RCC_APB2_Gate(RCC_APB2Periph_ADC1, DISABLE);
    HAL_RCC_APB2_Gate(RCC_APB2Periph_TIM1, DISABLE);

    // 3. 将所有与传感器相关的引脚置为高阻态(模拟输入)
    // 解释：这只能切断 STM32 内部的静态电流。由于硬件没有设计 MOSFET 开关，
    // MQ2 等外部模块本身的分压电阻依然会消耗微安级别的物理漏电流。
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 射频级低功耗：AT 模式下让 ESP8266 进入 modem sleep（自动休眠，有数据自动唤醒）。
    //    透传模式下 ESP 无法收 AT 命令，BSP_ESP8266_SetModemSleep 内部已按 g_esp_mode 拦截；
    //    flip-guard：仅在“首次进入休眠”时下发一次 AT+SLEEP= 1，避免每个空闲周期重复发命令。
    if (s_esp_lp_state == 0) {
        BSP_ESP8266_SetModemSleep(1);
        s_esp_lp_state = 1;
    }

    SYS_LOG("SENS", "Bus clock gated (ADC1/TIM1). Pins -> Analog Mode.\n");
}

void BSP_Sensors_Wakeup(void) {
    // 0. 先恢复总线时钟（依赖顺序：时钟 -> GPIO -> 外设 -> DMA -> 中断）
    HAL_RCC_APB2_Gate(RCC_APB2Periph_ADC1, ENABLE);
    HAL_RCC_APB2_Gate(RCC_APB2Periph_TIM1, ENABLE);

    // 1. 恢复引脚的原始工作模式
    HAL_GPIO_Init(GPIOA, GPIO_Pin_8, HAL_GPIO_MODE_OUTPUT_PP); // Trig
    HAL_TIM1_CH4_IC_Init(GPIOA, GPIO_Pin_11);                  // Echo (输入捕获重置)
    HAL_GPIO_Init(GPIOA, GPIO_Pin_15, HAL_GPIO_MODE_INPUT_PU); // DHT11

    // PA4 和 PA5 作为 ADC 原本就是 AIN 模式，无需重复设置

    // 2. 重新开启并强制校准 ADC
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // 3. 恢复定时器
    TIM_Cmd(TIM1, ENABLE);

    // 4. 射频级唤醒：退出 ESP8266 modem sleep（恢复全速射频，断网会话保持）。
    //    仅当本次确实有下发过休眠时才下发唤醒，且与 Sleep 的 flip-guard 配对。
    if (s_esp_lp_state) {
        BSP_ESP8266_SetModemSleep(0);
        s_esp_lp_state = 0;
    }
}

