#ifndef __HAL_PWM_H
#define __HAL_PWM_H

#include "stm32f10x.h"

// 舵机PWM配置参数
// 72MHz / 72 = 1MHz (1us/tick)
// ARR = 20000 - 1 -> 20ms (50Hz)
#define HAL_PWM_PRESCALER  (72 - 1)
#define HAL_PWM_PERIOD     (20000 - 1)

// 初始化定时器基准 (TIM2 或 TIM3)
void HAL_PWM_TimerInit(TIM_TypeDef* TIMx);

// 初始化 PWM 通道 (Channel 1-4)
void HAL_PWM_ConfigChannel(TIM_TypeDef* TIMx, uint8_t Channel);

// 设置比较值 (控制脉宽)
// Wrapper for TIM_SetCompareX
void HAL_PWM_SetCompare(TIM_TypeDef* TIMx, uint8_t Channel, uint16_t Compare);

#endif
