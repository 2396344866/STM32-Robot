#include "dev_hcsr04.h"
void Dev_HCSR04_Init(HCSR04_Handle_t* handle, void (*trig_fn)(uint8_t), void (*delay_fn)(uint32_t), uint32_t (*ic_fn)(void)) {
    handle->Trig_Write = trig_fn;
    handle->Delay_us = delay_fn;
    handle->GetPulseWidth = ic_fn;
}
float Dev_HCSR04_GetDistance(HCSR04_Handle_t* handle) {
    if (!handle->Trig_Write || !handle->GetPulseWidth) return -1.0f;
    
    handle->Trig_Write(1);
    handle->Delay_us(12); // 发送 >10us 高电平
    handle->Trig_Write(0);
    
    uint32_t pulse_width = handle->GetPulseWidth();
    return pulse_width * 0.017f; // us * 340m/s / 2
}
