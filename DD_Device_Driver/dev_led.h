#ifndef __DEV_LED_H
#define __DEV_LED_H

#include <stdint.h>
#include  <stddef.h>
// --- 1. 定义抽象接口 (V-Table) ---
// 写接口
// 1 = High Level, 0 = Low Level
typedef void (*LED_WritePin_Func)(uint8_t state); // state: 1=High Level, 0=Low Level
// 读接口：uint8_t (void)
// 返回值：1 = High Level, 0 = Low Level
typedef uint8_t (*LED_ReadPin_Func)(void);
typedef struct {
    LED_WritePin_Func WritePin; // 写入引脚电平的接口
		LED_ReadPin_Func  ReadPin;
} LED_IO_Interface_t;

// --- 2. 定义设备句柄 ---
// 注意：这里没有 GPIO_TypeDef* 或 Pin 号，只有逻辑属性
typedef enum {
    LED_ACTIVE_LOW = 0,
    LED_ACTIVE_HIGH = 1
} LED_ActiveLevel_t;

typedef struct {
    LED_IO_Interface_t io;      // 持有底层接口的实现
    LED_ActiveLevel_t  active_level; // 逻辑属性：高亮还是低亮
    uint8_t            is_on;   // 内部状态记录 (可选，用于逻辑判断)
} LED_Handle_t;

// --- 3. 逻辑接口声明 ---
void Dev_LED_Init(LED_Handle_t* hLed); // 逻辑初始化
void Dev_LED_On(LED_Handle_t* hLed);
void Dev_LED_Off(LED_Handle_t* hLed);
void Dev_LED_Toggle(LED_Handle_t* hLed);

#endif
