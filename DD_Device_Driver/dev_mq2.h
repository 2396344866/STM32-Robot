// dev_mq2.h
#ifndef __DEV_MQ2_H
#define __DEV_MQ2_H
#include <stdint.h>

typedef struct {
    uint16_t (*ReadADC)(void); // 依赖注入：ADC读取函数
} MQ2_Handle_t;

void Dev_MQ2_Init(MQ2_Handle_t* handle, uint16_t (*read_fn)(void));
float Dev_MQ2_GetPPM(MQ2_Handle_t* handle);
#endif

