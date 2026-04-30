#ifndef __HAL_SOFT_I2C_H
#define __HAL_SOFT_I2C_H

#include <stdint.h>
#include "hal_gpio.h" 

// 定义 I2C 句柄，支持多路 I2C 复用
typedef struct {
    GPIO_TypeDef* port_scl;
    uint16_t      pin_scl;
    GPIO_TypeDef* port_sda;
    uint16_t      pin_sda;
} SoftI2C_Handle_t;

// API 接口
void HAL_SoftI2C_Init(SoftI2C_Handle_t* hI2c);
void HAL_SoftI2C_Start(SoftI2C_Handle_t* hI2c);
void HAL_SoftI2C_Stop(SoftI2C_Handle_t* hI2c);
void HAL_SoftI2C_SendByte(SoftI2C_Handle_t* hI2c, uint8_t byte);
// 如果需要读取或检测 ACK，可在此扩展
// uint8_t HAL_SoftI2C_WaitAck(SoftI2C_Handle_t* hI2c);

#endif
