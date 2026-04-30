#include "dev_led.h"

void Dev_LED_Init(LED_Handle_t* hLed)
{
    // 防御性编程：检查接口是否注入成功
    if (!hLed || !hLed->io.WritePin) return;

    // 初始状态设为关闭
    Dev_LED_Off(hLed);
}

void Dev_LED_On(LED_Handle_t* hLed)
{
    if (!hLed || !hLed->io.WritePin) return;

    // 逻辑推演：如果是高电平亮，On 就是写 1；否则写 0
    uint8_t logic_level = (hLed->active_level == LED_ACTIVE_HIGH) ? 1 : 0;
    
    hLed->io.WritePin(logic_level);
    hLed->is_on = 1;
}

void Dev_LED_Off(LED_Handle_t* hLed)
{
    if (!hLed || !hLed->io.WritePin) return;

    // 逻辑推演：关灯电平与开灯相反
    uint8_t logic_level = (hLed->active_level == LED_ACTIVE_HIGH) ? 0 : 1;
    
    hLed->io.WritePin(logic_level);
    hLed->is_on = 0;
}

void Dev_LED_Toggle(LED_Handle_t* hLed)
{
    // 防御性编程
    if (!hLed || !hLed->io.WritePin) return;

    // 策略分发：优先使用硬件读取翻转，否则使用软件状态翻转
    if (hLed->io.ReadPin != NULL) {
        // --- 方案 A：基于硬件状态翻转 (更可靠) ---
        // 1. 读取当前物理电平
        uint8_t current_level = hLed->io.ReadPin();
        
        // 2. 计算反向电平 (逻辑非)
        uint8_t next_level = (current_level == 1) ? 0 : 1;
        
        // 3. 写入新电平
        hLed->io.WritePin(!current_level );
        
        // 4. 同步软件状态
        uint8_t physical_on_level = (hLed->active_level == LED_ACTIVE_HIGH) ? 1 : 0;
        hLed->is_on = (next_level == physical_on_level) ? 1 : 0;

    } else {
        // --- 方案 B：基于软件状态翻转 (降级模式) ---
        if (hLed->is_on) {
            Dev_LED_Off(hLed);
        } else {
            Dev_LED_On(hLed);
        }
    }
}
