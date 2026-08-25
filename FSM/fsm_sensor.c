#include "fsm_sensor.h"
#include "event_bus.h"
#include "sys_events.h"
#include "sys_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "bsp_sensor.h"
#include "hal_hw_cfg.h"  // 魔法数字宏 + assert_param

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
    float dist = 0.0f, smoke = 0.0f, lux = 0.0f;
    uint8_t t = 0, h = 0;

    // 1. 读取超声波
    dist = BSP_Sensor_GetDistance();

    // 避障逻辑判断（本地副本，不持锁）
    if (dist > HAL_HW_CFG_AVOID_NEAR_MIN_CM && dist < HAL_HW_CFG_AVOID_NEAR_MAX_CM) {
        if (!is_obstacle_blocked) {
            is_obstacle_blocked = 1;
            SYS_LOG("SENS", "Obstacle Detected! Dist: %.1f cm\n", dist);
            event_bus_publish(EVT_WARN_OBSTACLE, 0);
        }
    } else if (dist >= HAL_HW_CFG_AVOID_CLEAR_CM) {
        if (is_obstacle_blocked) {
            is_obstacle_blocked = 0;
            SYS_LOG("SENS", "Obstacle Cleared! Dist: %.1f cm\n", dist);
            event_bus_publish(EVT_OBSTACLE_CLEARED, 0);
        }
    }

    // 2. 读取模拟量（从 ADC 双缓冲非活跃半区同步最新帧）
    BSP_Sensor_SyncAdcFrame();
    smoke = BSP_Sensor_GetSmoke();
    lux  = BSP_Sensor_GetLight();

    // 3. 读取 DHT11 (降频读取)
    uint8_t dht_ok = 0;
    if (++dht_div >= HAL_HW_CFG_DHT11_POLL_DIV) {
        dht_div = 0;
        if (BSP_Sensor_ReadDHT11(&t, &h)) {
            dht_ok = 1;
        }
    }

    // 4. 统一在互斥量保护下提交一帧传感器数据（防 float 多字节读写撕裂）
    if (xSensorDataMutex != NULL && xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(HAL_HW_CFG_SENSOR_MUTEX_TIMEOUT_MS)) == pdPASS) {
        g_sensor_data.distance   = dist;
        g_sensor_data.smoke_ppm  = smoke;
        g_sensor_data.light_lux  = lux;
        if (dht_ok) {
            g_sensor_data.temp = (float)t;
            g_sensor_data.hum  = (float)h;
        }
        xSemaphoreGive(xSensorDataMutex);
    }
    if (dht_ok) {
        SYS_LOG("SENS", "Env Update -> T:%.1fC, H:%.1f%%, Smoke:%.1fppm, Light:%.1f Lux\n",
                g_sensor_data.temp, g_sensor_data.hum, g_sensor_data.smoke_ppm, g_sensor_data.light_lux);
    }
}



//// 状态机周期轮询回调
//static void on_poll_sensor(fsm_t* fsm, void* arg) {
//    static uint8_t dht_div = 0;
//    // 1. 读取超声波
//    float dist = BSP_Sensor_GetDistance();
//    g_sensor_data.distance = dist;
//    // 避障逻辑判断
//    if (dist > HAL_HW_CFG_AVOID_NEAR_MIN_CM && dist < HAL_HW_CFG_AVOID_NEAR_MAX_CM) {
//        if (!is_obstacle_blocked) {
//            is_obstacle_blocked = 1;
//            SYS_LOG("SENS", "Obstacle Detected! Dist: %.1f cm\n", dist); // 状态突变打印
//            event_bus_publish(EVT_WARN_OBSTACLE, 0); 
//        }
//    } else if (dist >= HAL_HW_CFG_AVOID_CLEAR_CM) { 
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
//    if (++dht_div >= HAL_HW_CFG_DHT11_POLL_DIV) {
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
