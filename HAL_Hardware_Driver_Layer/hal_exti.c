#include "hal_exti.h"

static EXTI_Callback_t PB12_Callback = 0;

void HAL_EXTI_Init_PB12(void) {
    GPIO_InitTypeDef GPIO_InitStruct;    
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    
    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5; // 根据FreeRTOS需求调整
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void HAL_EXTI_RegisterCallback_PB12(EXTI_Callback_t callback) {
    PB12_Callback = callback;
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        if (PB12_Callback != 0) {
            PB12_Callback();
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
