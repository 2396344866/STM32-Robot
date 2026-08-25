#ifndef __HAL_RCC_H
#define __HAL_RCC_H

#include "stm32f10x.h"

/*
 * @brief 总线级时钟门控 (Bus Clock Gating)
 *
 * 与 bsp_sensor.c 中“外设使能位门控(ADC_Cmd/TIM_Cmd)”不同，这里关闭的是
 * 整条 AHB/APB 总线的时钟 (RCC_AHB/APBxPeriphClockCmd DISABLE)。
 * 关闭后该总线上所有挂载外设的寄存器访问都将失效，直至重新 ENABLE，
 * 因此可进一步压低睡眠功耗——这是简历中“动态时钟门控”的准确实现。
 *
 * 约束：
 *  - 关闭总线前必须确保无外设正在发起总线访问（DMA/中断已停），否则 HardFault；
 *  - 唤醒时必须按 时钟→GPIO→外设→DMA→中断 的依赖顺序恢复（见 hal_adc_dma / hal_gpio）。
 */

/* ---- APB2（高速外设总线，挂载 ADC1/USART1/GPIOA..D/ TIM1 等） ---- */
static inline void HAL_RCC_APB2_Gate(uint32_t periph, FunctionalState state) {
    RCC_APB2PeriphClockCmd(periph, state);
}

/* ---- APB1（低速外设总线，挂载 I2C2/TIM2..7/USART2/3 等） ---- */
static inline void HAL_RCC_APB1_Gate(uint32_t periph, FunctionalState state) {
    RCC_APB1PeriphClockCmd(periph, state);
}

/* ---- AHB（DMA/SRAM/FLASH 等） ---- */
static inline void HAL_RCC_AHB_Gate(uint32_t periph, FunctionalState state) {
    RCC_AHBPeriphClockCmd(periph, state);
}

#endif /* __HAL_RCC_H */
