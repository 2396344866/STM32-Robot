// dev_dht11.h
#ifndef __DEV_DHT11_H
#define __DEV_DHT11_H
#include <stdint.h>

typedef struct {
    void (*SetPinOut)(void);
    void (*SetPinIn)(void);
    void (*WritePin)(uint8_t state);
    uint8_t (*ReadPin)(void);
    void (*DelayUs)(uint32_t us);
    void (*DelayMs)(uint32_t ms);
} DHT11_Handle_t;

void Dev_DHT11_Init(DHT11_Handle_t* handle);
uint8_t Dev_DHT11_Read(DHT11_Handle_t* handle, uint8_t *temp, uint8_t *humi);
#endif
