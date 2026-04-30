#ifndef FSM_SENSOR_H
#define FSM_SENSOR_H

#include "fsm_core.h"
#include <stdint.h>

// 全局传感器数据结构
typedef struct {
    float   temp;           // DHT11 温度
    float   hum;            // DHT11 湿度
    float   smoke_ppm;      // 烟雾 ADC 浓度
    float   distance;       // 超声波距离 (cm)
    float   light_lux;      // 预留光照
} SensorData_t;

// 供全网其他模块读取的全局数据
extern SensorData_t g_sensor_data;

void Sensor_FSM_Task(void *pvParameters);

#endif
