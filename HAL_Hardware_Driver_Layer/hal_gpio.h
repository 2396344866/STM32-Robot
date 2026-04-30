#ifndef __HAL_GPIO_H
#define __HAL_GPIO_H

#include "stm32f10x.h"
#include <stdbool.h>
// 定义引脚电平
typedef enum {
    HAL_PIN_RESET = 0,
    HAL_PIN_SET   = 1
  } HAL_PinState_t;
// 定义引脚模式 (简化版，覆盖你的工程需求)
typedef enum {
    HAL_GPIO_MODE_INPUT_PU = GPIO_Mode_IPU,    // 上拉输入 (按键)
		HAL_GPIO_MODE_INPUT_PD = GPIO_Mode_IPD,    // 下拉输入 (高电平有效按键)
    HAL_GPIO_MODE_OUTPUT_PP = GPIO_Mode_Out_PP,// 推挽输出 (LED)
    HAL_GPIO_MODE_OUTPUT_OD = GPIO_Mode_Out_OD,// 开漏输出 (软件I2C)
    HAL_GPIO_MODE_AF_PP     = GPIO_Mode_AF_PP,  // 复用推挽 (PWM)
		HAL_GPIO_MODE_INPUT_FLOATING    = GPIO_Mode_IN_FLOATING, //浮空输入
		
} HAL_GpioMode_t;
// --- 接口声明 ---
// 初始化指定引脚 (自动开启时钟)
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, HAL_GpioMode_t Mode);
// 写引脚
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, HAL_PinState_t PinState);
// 翻转引脚 (用于LED闪烁)
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
// 读引脚
HAL_PinState_t HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif
