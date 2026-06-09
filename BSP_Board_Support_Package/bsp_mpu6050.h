#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include <stdint.h>

// MPU6050 姿态数据核心结构体
typedef struct {
    float pitch;
    float roll;
    float yaw;
    short gyro[3];
    short accel[3];
} MPU6050_Data_t;

void BSP_MPU6050_Init(void);
int  BSP_MPU6050_GetData(MPU6050_Data_t *data);
uint8_t BSP_MPU6050_IsWorking(void);

// ========================================================
// 桥接声明：专门暴露给底层第三方 DMP 库 (inv_mpu.c) 的物理接口
// ========================================================
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
void Delay_ms(uint32_t ms);
void get_tick_count(unsigned long *count);

// ========================================================
// 日志声明：接管所有 MPU 相关的底层打印
// ========================================================
void BSP_MPL_LOGI(const char* fmt, ...);
void BSP_MPL_LOGE(const char* fmt, ...);

#endif
