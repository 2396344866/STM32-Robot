#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "dev_mpu6050.h"

void BSP_MPU6050_Init(void);
int BSP_MPU6050_GetData(MPU6050_Data_t *data);
uint8_t BSP_MPU6050_IsDataReady(void);
void BSP_MPU6050_ClearDataReady(void);
uint8_t BSP_MPU6050_IsWorking(void);
void BSP_MPL_LOGI(const char* fmt, ...);
void BSP_MPL_LOGE(const char* fmt, ...);
#endif
