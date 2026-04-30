#include "hal_tim_ic.h"

// 内部状态机变量
static volatile uint32_t tim1_ic_val1 = 0, tim1_ic_val2 = 0;
static volatile uint8_t  tim1_ic_edge = 0;
static volatile uint32_t tim1_pulse_width = 0;

void HAL_TIM1_CH4_IC_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    if (GPIOx == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

uint32_t HAL_TIM1_CH4_GetPulseWidth(void) {
    return tim1_pulse_width;
}

// 硬件中断服务函数
void TIM1_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET) {
        if (tim1_ic_edge == 0) { 
            tim1_ic_val1 = TIM_GetCapture4(TIM1);
            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling); 
            tim1_ic_edge = 1;
        } else { 
            tim1_ic_val2 = TIM_GetCapture4(TIM1);
            if (tim1_ic_val2 >= tim1_ic_val1) {
                tim1_pulse_width = tim1_ic_val2 - tim1_ic_val1;
            } else {
                tim1_pulse_width = 0xFFFF - tim1_ic_val1 + tim1_ic_val2;
            }
            TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); 
            tim1_ic_edge = 0;
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
    }
}
