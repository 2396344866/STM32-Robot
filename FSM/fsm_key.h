/* key.h */
#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

/* ================== 1. 队列句柄声明 ================== */
extern QueueHandle_t xKeyLogicQueue;

/* ================== 2. 引脚定义 ================== */
// K1: 模式选择 (PA12)
#define KEY1_GPIO_CLK  RCC_APB2Periph_GPIOA
#define KEY1_GPIO_PORT GPIOA
#define KEY1_GPIO_PIN  GPIO_Pin_12

// K2: 确认/停止 (PB13)
#define KEY2_GPIO_CLK  RCC_APB2Periph_GPIOB
#define KEY2_GPIO_PORT GPIOB
#define KEY2_GPIO_PIN  GPIO_Pin_13

/* ================== 3. 扫描参数配置 ================== */
#define KEY_SCAN_PERIOD_MS  10      // 扫描周期 10ms
#define KEY_DEBOUNCE_MS     20      // 消抖时间 20ms
#define KEY_LONG_PRESS_MS   800     // 长按时间 800ms

// 计算 Tick 数 (基于扫描周期)
#define KEY_DEBOUNCE_TICKS  (KEY_DEBOUNCE_MS / KEY_SCAN_PERIOD_MS)  //短按次数
#define KEY_LONG_TICKS      (KEY_LONG_PRESS_MS / KEY_SCAN_PERIOD_MS) //长按次数

/* 初始化函数 */
void KEY_Init(void);

#endif
