// dev_hcsr04.h
#ifndef __DEV_HCSR04_H
#define __DEV_HCSR04_H
#include <stdint.h>

typedef struct {
    void (*Trig_Write)(uint8_t state);
    void (*Delay_us)(uint32_t us);
    uint32_t (*GetPulseWidth)(void);
} HCSR04_Handle_t;

void Dev_HCSR04_Init(HCSR04_Handle_t* handle, void (*trig_fn)(uint8_t), void (*delay_fn)(uint32_t), uint32_t (*ic_fn)(void));
float Dev_HCSR04_GetDistance(HCSR04_Handle_t* handle);
#endif
