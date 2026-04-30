#include "hal_gpio.h"

// 内部辅助函数：根据GPIO端口自动使能时钟
static void HAL_GPIO_EnableClock(GPIO_TypeDef* GPIOx)
{
    if (GPIOx == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (GPIOx == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, HAL_GpioMode_t Mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 自动开启时钟
    HAL_GPIO_EnableClock(GPIOx);

    // 2. 配置结构体
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)Mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // 3. 初始化
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, HAL_PinState_t PinState)
{
    if (PinState == HAL_PIN_SET) {
        GPIOx->BSRR = GPIO_Pin; // 置位
    } else {
        GPIOx->BRR = GPIO_Pin;  // 复位
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // 读取ODR寄存器对应位，异或操作实现翻转
    uint16_t outputData = GPIO_ReadOutputData(GPIOx);
    if ((outputData & GPIO_Pin) != 0) {
        GPIOx->BRR = GPIO_Pin;
    } else {
        GPIOx->BSRR = GPIO_Pin;
    }
}

HAL_PinState_t HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == Bit_SET) ? HAL_PIN_SET : HAL_PIN_RESET;
}
