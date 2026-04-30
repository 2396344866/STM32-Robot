#include "fsm_sensor.h"
#include "event_bus.h"
#include "sys_events.h"
#include "sys_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_sensor.h" // BSP层集成头文件

SensorData_t g_sensor_data = {0};
static fsm_t g_sensor_fsm;
static uint8_t is_obstacle_blocked = 0; 

// 状态机周期轮询回调
static void on_poll_sensor(fsm_t* fsm, void* arg) {
    static uint8_t dht_div = 0;
    // 1. 读取超声波
    float dist = BSP_Sensor_GetDistance();
    g_sensor_data.distance = dist;
    // 避障逻辑判断
    if (dist > 2.0f && dist < 15.0f) {
        if (!is_obstacle_blocked) {
            is_obstacle_blocked = 1;
            SYS_LOG("SENS", "Obstacle Detected! Dist: %.1f cm\n", dist); // 状态突变打印
            event_bus_publish(EVT_WARN_OBSTACLE, 0); 
        }
    } else if (dist >= 17.0f) { 
        if (is_obstacle_blocked) {
            is_obstacle_blocked = 0;
            SYS_LOG("SENS", "Obstacle Cleared! Dist: %.1f cm\n", dist);  // 状态突变打印
            event_bus_publish(EVT_OBSTACLE_CLEARED, 0);
        }
    }
    // 2. 读取烟雾 ADC
    g_sensor_data.smoke_ppm = BSP_Sensor_GetSmoke();
		// 【新增】读取光照数据
    g_sensor_data.light_lux = BSP_Sensor_GetLight();
		
		
    // 3. 读取 DHT11 (降频读取)
    if (++dht_div >= 20) {
        dht_div = 0;
        uint8_t t, h;
        if (BSP_Sensor_ReadDHT11(&t, &h)) { 
            g_sensor_data.temp = (float)t;
            g_sensor_data.hum = (float)h;
            // 去除百分号，更改为 Lux 单位
            SYS_LOG("SENS", "Env Update -> T:%.1fC, H:%.1f%%, Smoke:%.1fppm, Light:%.1f Lux\n", 
                    g_sensor_data.temp, g_sensor_data.hum, g_sensor_data.smoke_ppm, g_sensor_data.light_lux);
        } else {
            SYS_LOG("SENS", "DHT11 Read Failed (Preempted)\n");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
}

static const fsm_state_desc_t sensor_states[] = {
    { 0, NULL, NULL, on_poll_sensor } // 单一状态，完全靠 poll 驱动
};
static const fsm_transition_t sensor_trans[] = { {0, 0, 0, NULL, NULL} }; // 无迁移

void Sensor_FSM_Task(void *pvParameters) {
    BSP_Sensors_Init(); // 底层硬件初始化
    
    fsm_init(&g_sensor_fsm, NULL, 0, sensor_trans, 0, 0, NULL);
    fsm_set_state_callbacks(&g_sensor_fsm, sensor_states, 1);
    
    while(1) {
        fsm_run(&g_sensor_fsm);
    }
}
