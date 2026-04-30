#include "hal_pwm.h"

// 内部辅助：开启定时器时钟
static void HAL_PWM_EnableTimerClock(TIM_TypeDef* TIMx)
{
    if (TIMx == TIM2) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    else if (TIMx == TIM3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    else if (TIMx == TIM4) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 新增 TIM4 时钟使能
}

void HAL_PWM_TimerInit(TIM_TypeDef* TIMx)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 1. 开启时钟
    HAL_PWM_EnableTimerClock(TIMx);

    // 2. 配置时基 (针对舵机优化的 50Hz)
    TIM_TimeBaseStructure.TIM_Period = HAL_PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = HAL_PWM_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    // 3. 开启定时器
    TIM_Cmd(TIMx, ENABLE);
}

void HAL_PWM_ConfigChannel(TIM_TypeDef* TIMx, uint8_t Channel)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    switch (Channel) {
        case 1: TIM_OC1Init(TIMx, &TIM_OCInitStructure); break;
        case 2: TIM_OC2Init(TIMx, &TIM_OCInitStructure); break;
        case 3: TIM_OC3Init(TIMx, &TIM_OCInitStructure); break;
        case 4: TIM_OC4Init(TIMx, &TIM_OCInitStructure); break;
        default: break;
    }
    
    // 使能预装载寄存器
    switch (Channel) {
        case 1: TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
        case 2: TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
        case 3: TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
        case 4: TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
    }
}

void HAL_PWM_SetCompare(TIM_TypeDef* TIMx, uint8_t Channel, uint16_t Compare)
{
    switch (Channel) {
        case 1: TIM_SetCompare1(TIMx, Compare); break;
        case 2: TIM_SetCompare2(TIMx, Compare); break;
        case 3: TIM_SetCompare3(TIMx, Compare); break;
        case 4: TIM_SetCompare4(TIMx, Compare); break;
        default: break;
    }
}
