// dev_mq2.c
#include "dev_mq2.h"
void Dev_MQ2_Init(MQ2_Handle_t* handle, uint16_t (*read_fn)(void)) {
    handle->ReadADC = read_fn;
}
float Dev_MQ2_GetPPM(MQ2_Handle_t* handle) {
    if (!handle->ReadADC) return 0.0f;
    uint16_t raw_adc = handle->ReadADC();
    float vol = (float)raw_adc * (3.3f / 4096.0f);
    return vol * 100.0f; // 线性映射示例
}
