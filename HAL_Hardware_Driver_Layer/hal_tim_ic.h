#ifndef __HAL_TIM_IC_H
#define __HAL_TIM_IC_H
#include "stm32f10x.h"

void HAL_TIM1_CH4_IC_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint32_t HAL_TIM1_CH4_GetPulseWidth(void);

#endif
