#ifndef __DEV_MPU6050_H
#define __DEV_MPU6050_H

#include <stdint.h>

// 1. 定义设备依赖接口 (V-Table)
typedef struct {
    void (*InitIO)(void);
    int  (*WriteMem)(uint8_t DevAddr, uint8_t RegAddr, uint8_t *Data, uint16_t Size);
    int  (*ReadMem)(uint8_t DevAddr, uint8_t RegAddr, uint8_t *Data, uint16_t Size);
    void (*DelayMs)(uint32_t ms);
    void (*GetTick)(uint32_t *timestamp);
} MPU6050_IO_Interface_t;

typedef struct {
    MPU6050_IO_Interface_t io;
} MPU6050_Handle_t;

// 2. 数据结构体
typedef struct {
    float pitch;
    float roll;
    float yaw;
    short gyro[3];
    short accel[3];
} MPU6050_Data_t;

// 3. API
int Dev_MPU6050_Init(MPU6050_Handle_t *handle);
int Dev_MPU6050_Read_DMP(MPU6050_Data_t *out_data);
void get_tick_count(unsigned long *count);
void Dev_MPU6050_RunCalibration(void);
#endif
