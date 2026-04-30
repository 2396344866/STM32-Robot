#ifndef __HAL_DELAY_H
#define __HAL_DELAY_H

#include "stm32f10x.h"

/* ==========================================
 * 配置宏
 * ========================================== */
// 如果工程中定义了使用 FreeRTOS，则开启相关支持
// 通常在编译器预定义宏或 FreeRTOSConfig.h 中定义，这里默认开启以匹配你的需求
#define HAL_USE_FREERTOS 

/* ==========================================
 * 接口声明
 * ========================================== */

/**
 * @brief  初始化延时模块 (计算时钟频率，初始化倍乘因子)
 * @note   必须在系统时钟配置完成后调用
 */
void HAL_Delay_Init(void);

/**
 * @brief  微秒级延时 (阻塞式)
 * @note   基于 SysTick 计数器的差值计算，不占用中断，不干扰 OS 节拍
 * 适用于 < 1ms 的短延时或关中断场景
 */
void HAL_Delay_us(uint32_t us);

/**
 * @brief  毫秒级延时 (智能模式)
 * @note   - 若 FreeRTOS 运行中且不在中断内：调用 vTaskDelay (释放 CPU)
 * - 若 FreeRTOS 未启动或在中断内：调用微秒级死循环 (占用 CPU)
 */
void HAL_Delay_ms(uint32_t ms);

/**
 * @brief  获取系统运行时间戳 (ms)
 * @return 若 OS 运行则返回 OS Tick，否则返回 0
 */
uint32_t HAL_GetTick(void);

#endif
