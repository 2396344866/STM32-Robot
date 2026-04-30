#ifndef __HAL_EXTI_H
#define __HAL_EXTI_H

#include "stm32f10x.h"

// 定义回调函数指针类型
typedef void (*EXTI_Callback_t)(void);

void HAL_EXTI_Init_PB12(void);
void HAL_EXTI_RegisterCallback_PB12(EXTI_Callback_t callback);

#endif
