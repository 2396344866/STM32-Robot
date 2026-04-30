#include "fsm_sensor.h"
#include "event_bus.h"
#include "sys_events.h"
#include "sys_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_sensor.h"

SensorData_t g_sensor_data = {0};
static fsm_t g_sensor_fsm;
static uint8_t is_obstacle_blocked = 0; 

// 为传感器状态机分配事件队列内存（需接收网络事件）
static fsm_event_t sensor_evt_buf[8];
// --- 状态枚举 ---
typedef enum {
    STATE_SENS_SLEEP = 0,
    STATE_SENS_ACTIVE
} sensor_state_t;
// --- 回调函数 ---
static void on_enter_sleep(fsm_t* fsm, void* arg) {
    BSP_Sensors_Sleep();
    SYS_LOG("SENS", "Enter SLEEP Mode. Hardware Paused.\n");
}

static void on_enter_active(fsm_t* fsm, void* arg) {
    BSP_Sensors_Wakeup();
    SYS_LOG("SENS", "Enter ACTIVE Mode. Hardware Running.\n");
}

// 轮询采集函数 (仅在 ACTIVE 状态下执行)
static void on_poll_active(fsm_t* fsm, void* arg) {
    static uint8_t dht_div = 0;
    
    // 1. 读取超声波
    float dist = BSP_Sensor_GetDistance();
    g_sensor_data.distance = dist;
    
    // 避障逻辑判断
    if (dist > 2.0f && dist < 15.0f) {
        if (!is_obstacle_blocked) {
            is_obstacle_blocked = 1;
            SYS_LOG("SENS", "Obstacle Detected! Dist: %.1f cm\n", dist);
            event_bus_publish(EVT_WARN_OBSTACLE, 0); 
        }
    } else if (dist >= 17.0f) { 
        if (is_obstacle_blocked) {
            is_obstacle_blocked = 0;
            SYS_LOG("SENS", "Obstacle Cleared! Dist: %.1f cm\n", dist); 
            event_bus_publish(EVT_OBSTACLE_CLEARED, 0);
        }
    }
    
    // 2. 读取模拟量
    g_sensor_data.smoke_ppm = BSP_Sensor_GetSmoke();
    g_sensor_data.light_lux = BSP_Sensor_GetLight();
		
    // 3. 读取 DHT11 (降频读取)
    if (++dht_div >= 20) {
        dht_div = 0;
        uint8_t t, h;
        if (BSP_Sensor_ReadDHT11(&t, &h)) { 
            g_sensor_data.temp = (float)t;
            g_sensor_data.hum = (float)h;
            SYS_LOG("SENS", "Env Update -> T:%.1fC, H:%.1f%%, Smoke:%.1fppm, Light:%.1f Lux\n", 
                    g_sensor_data.temp, g_sensor_data.hum, g_sensor_data.smoke_ppm, g_sensor_data.light_lux);
        }
    }
}



//// 状态机周期轮询回调
//static void on_poll_sensor(fsm_t* fsm, void* arg) {
//    static uint8_t dht_div = 0;
//    // 1. 读取超声波
//    float dist = BSP_Sensor_GetDistance();
//    g_sensor_data.distance = dist;
//    // 避障逻辑判断
//    if (dist > 2.0f && dist < 15.0f) {
//        if (!is_obstacle_blocked) {
//            is_obstacle_blocked = 1;
//            SYS_LOG("SENS", "Obstacle Detected! Dist: %.1f cm\n", dist); // 状态突变打印
//            event_bus_publish(EVT_WARN_OBSTACLE, 0); 
//        }
//    } else if (dist >= 17.0f) { 
//        if (is_obstacle_blocked) {
//            is_obstacle_blocked = 0;
//            SYS_LOG("SENS", "Obstacle Cleared! Dist: %.1f cm\n", dist);  // 状态突变打印
//            event_bus_publish(EVT_OBSTACLE_CLEARED, 0);
//        }
//    }
//    // 2. 读取烟雾 ADC
//    g_sensor_data.smoke_ppm = BSP_Sensor_GetSmoke();
//		// 【新增】读取光照数据
//    g_sensor_data.light_lux = BSP_Sensor_GetLight();
//		
//		
//    // 3. 读取 DHT11 (降频读取)
//    if (++dht_div >= 20) {
//        dht_div = 0;
//        uint8_t t, h;
//        if (BSP_Sensor_ReadDHT11(&t, &h)) { 
//            g_sensor_data.temp = (float)t;
//            g_sensor_data.hum = (float)h;
//            // 去除百分号，更改为 Lux 单位
//            SYS_LOG("SENS", "Env Update -> T:%.1fC, H:%.1f%%, Smoke:%.1fppm, Light:%.1f Lux\n", 
//                    g_sensor_data.temp, g_sensor_data.hum, g_sensor_data.smoke_ppm, g_sensor_data.light_lux);
//        } else {
//            SYS_LOG("SENS", "DHT11 Read Failed (Preempted)\n");
//        }
//    }
//    vTaskDelay(pdMS_TO_TICKS(100)); 
//}

static const fsm_state_desc_t sensor_states[] = {
    { STATE_SENS_SLEEP,  on_enter_sleep,  NULL, NULL },
    { STATE_SENS_ACTIVE, on_enter_active, NULL, on_poll_active } 
};

static const fsm_transition_t sensor_trans[] = {
    // 收到网络上线事件，进入采集状态
    { STATE_SENS_SLEEP,  EVT_NET_STATUS_ONLINE, STATE_SENS_ACTIVE, NULL, NULL },
    // 网络断开或出错，退回休眠状态
    { STATE_SENS_ACTIVE, EVT_NET_STATUS_ERROR,  STATE_SENS_SLEEP,  NULL, NULL },
    { STATE_SENS_ACTIVE, EVT_NET_STATUS_INIT,   STATE_SENS_SLEEP,  NULL, NULL }
};

void Sensor_FSM_Task(void *pvParameters) {
    BSP_Sensors_Init(); 
    
    // 绑定上下文与迁移规则。初始状态强制设置为 SLEEP。
    fsm_init(&g_sensor_fsm, sensor_evt_buf, 8, 
             sensor_trans, sizeof(sensor_trans)/sizeof(fsm_transition_t), 
             STATE_SENS_SLEEP, NULL);
             
    fsm_set_state_callbacks(&g_sensor_fsm, sensor_states, 2);
    
    // 订阅网络状态事件，接收外网 FSM 的指挥
    event_bus_subscribe(&g_sensor_fsm, EVT_NET_STATUS_ONLINE);
    event_bus_subscribe(&g_sensor_fsm, EVT_NET_STATUS_ERROR);
    event_bus_subscribe(&g_sensor_fsm, EVT_NET_STATUS_INIT);
    
    // 主动触发第一次休眠动作
    on_enter_sleep(&g_sensor_fsm, NULL);

    while(1) {
        fsm_run(&g_sensor_fsm);
        
        // 动态调频机制：休眠状态下任务大周期挂起让出CPU，活跃状态下保持100ms周期
        if (g_sensor_fsm.current_state == STATE_SENS_SLEEP) {
            vTaskDelay(pdMS_TO_TICKS(500)); 
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
    }
}
