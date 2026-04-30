#include "dev_dht11.h"

void Dev_DHT11_Init(DHT11_Handle_t* handle) {
    // 句柄指针由外部注入，内部不执行硬件操作
}

uint8_t Dev_DHT11_Read(DHT11_Handle_t* handle, uint8_t *temp, uint8_t *humi) {
    uint8_t buf[5] = {0};
    uint8_t i, j;
    uint16_t timeout;
    
    handle->SetPinOut();
    handle->WritePin(0);
    handle->DelayMs(20);
    handle->WritePin(1);
    handle->DelayUs(30);
    
    handle->SetPinIn();
    
    timeout = 100;
    while(handle->ReadPin() == 1 && timeout--) handle->DelayUs(1);
    if(timeout == 0) return 0;
    
    timeout = 100;
    while(handle->ReadPin() == 0 && timeout--) handle->DelayUs(1);
    if(timeout == 0) return 0;
    
    timeout = 100;
    while(handle->ReadPin() == 1 && timeout--) handle->DelayUs(1);
    
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            timeout = 100;
            while(handle->ReadPin() == 0 && timeout--) handle->DelayUs(1);
            
            handle->DelayUs(40);
            buf[i] <<= 1;
            
            if (handle->ReadPin() == 1) {
                buf[i] |= 1;
                timeout = 100;
                while(handle->ReadPin() == 1 && timeout--) handle->DelayUs(1);
            }
        }
    }
    
    if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
        *humi = buf[0];
        *temp = buf[2];
        return 1;
    }
    return 0;
}
