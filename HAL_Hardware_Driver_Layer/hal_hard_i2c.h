#ifndef __HAL_HARD_I2C_H
#define __HAL_HARD_I2C_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
    I2C_TypeDef* I2Cx;
    GPIO_TypeDef* GPIO_Port;
    uint16_t      SCL_Pin;
    uint16_t      SDA_Pin;
    uint32_t      ClockSpeed;
} HardI2C_Handle_t;

void HAL_HardI2C_Init(HardI2C_Handle_t *hI2c);
void HAL_HardI2C_ResetBus(HardI2C_Handle_t *hI2c);
int HAL_HardI2C_WriteMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size);
int HAL_HardI2C_ReadMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size);

#endif
