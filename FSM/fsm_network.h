#ifndef FSM_NETWORK_H
#define FSM_NETWORK_H

#include "fsm_core.h"
#include <stdint.h>

/* --- 全局传感器数据结构 --- */
typedef struct {
    float temp;
    float hum;
    float light_lux;
    float smoke_ppm;
} GlobalSensorData_t;

extern GlobalSensorData_t g_SensorData;

/* --- 网络状态机定义 --- */
typedef enum {
    STATE_NET_INIT = 0,
    STATE_NET_WIFI_CONN,
    STATE_NET_MQTT_CONN,
    STATE_NET_ONLINE,
    STATE_NET_ERROR,
    NET_STATE_MAX
} Net_System_state_t;

typedef struct {
    uint8_t init_step;
    uint32_t last_tx_tick;
		// 是否开启欧拉角上传数据
		uint8_t euler_report_en;
} Net_context_t;

void Network_FSM_Setup(fsm_t* fsm);
void Network_FSM_Task(void *pvParameters);

#endif
