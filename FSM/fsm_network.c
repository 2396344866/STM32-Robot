#include "fsm_network.h"
#include "event_bus.h"
#include "sys_events.h"
#include "bsp_esp8266.h"
#include "sys_config.h"
#include <string.h>
#include <stdio.h>
#include "bsp_debug_uart.h"
#include "sys_config.h"
#include "bsp_mpu6050.h"
#include "fsm_motor.h"
#include "fsm_sensor.h"
// 1. 变量定义处
#if ENABLE_DEBUG_PRINT
static uint8_t g_fsm_paused = 0;
#endif
#define WIFI_SSID       "yunyan105"
#define WIFI_PWD        "88888888"
#define USERNAME        "STM32_dev&k1tjfOebU45"
#define PASSWORD        "b89f19fe9a381fd85c55538162140a7fe4327186c0bd8b808ad21fe342f9d642"
#define CLIENTID        "k1tjfOebU45.STM32_dev|securemode=2\\,signmethod=hmacsha256\\,timestamp=1773839733316|"
#define DOMAINNAME      "iot-06z00humnptvgav.mqtt.iothub.aliyuncs.com"
#define DEVICENAME      "STM32_dev"
#define PRODUCTKEY      "k1tjfOebU45" 


static fsm_event_t net_evt_buffer[16];
static Net_context_t net_ctx;
//GlobalSensorData_t g_SensorData;
#define APP_SOURCE 2
#define ENCODE_PARAM(source, evt) (((uint32_t)(source) << 16) | (evt))

// --- JSON 解析 (第一性原理：底层子串匹配) ---
static void parse_aliyun_payload(const char* payload) {
    uint16_t target_evt = EVT_NONE;

    // 1. 匹配动作指令 (value:1)
    if (strstr(payload, "\"value\":1")) {
        if (strstr(payload, "\"move_on\""))             target_evt = EVT_MOTOR_FORWARD;
        else if (strstr(payload, "\"move_back\""))      target_evt = EVT_MOTOR_BACKWARD;
        else if (strstr(payload, "\"move_left\""))      target_evt = EVT_MOTOR_LEFT;
        else if (strstr(payload, "\"move_right\""))     target_evt = EVT_MOTOR_RIGHT;
        else if (strstr(payload, "\"move_left_rotate\"")) target_evt = EVT_MOTOR_ROT_L;
        else if (strstr(payload, "\"move_right_rotate\""))target_evt = EVT_MOTOR_ROT_R;
        else if (strstr(payload, "\"move_stop\""))      target_evt = EVT_MOTOR_STOP;
    } 
    // 2. 匹配复位/停止指令 (value:0)
    else if (strstr(payload, "\"value\":0")) {
        target_evt = EVT_MOTOR_STOP;
    }

    // 3. 发布带特征码的事件
    if (target_evt != EVT_NONE) {
        event_bus_publish(target_evt, ENCODE_PARAM(APP_SOURCE, target_evt));
    }
		if (strstr(payload, "\"Euler_angle_open\"")) {
				if (strstr(payload, "\"value\":1")) {
						event_bus_publish(EVT_NET_EULER_OPEN, 1);
				} else if (strstr(payload, "\"value\":0")) {
						event_bus_publish(EVT_NET_EULER_CLOSE, 0);
				}
		}
}


// --- 状态回调 ---
static void on_enter_init(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->init_step = 0;
    ctx->last_tx_tick = FSM_GET_TICK(); // 初始化时间基准
    event_bus_publish(EVT_NET_STATUS_INIT, 0);
}
static void on_poll_init(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
				if (g_fsm_paused) return; 
		#endif
    
    Net_context_t* ctx = (Net_context_t*)arg;
    uint32_t current_tick = FSM_GET_TICK();

    if (ctx->init_step == 0) {
        BSP_ESP8266_SendString("AT+RST\r\n");
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    else if (ctx->init_step == 1 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(1000))) {
        BSP_ESP8266_SendString("ATE0\r\n");
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    else if (ctx->init_step == 2 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(1000))) {
        BSP_ESP8266_SendString("AT+CWMODE=1\r\n");
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    else if (ctx->init_step == 3 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(1000))) {
        fsm_push_event(fsm, EVT_INIT_DONE, 0);
    }
}

static void on_enter_wifi(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    char buf[128];
    ctx->last_tx_tick = FSM_GET_TICK(); // 记录起始时间戳以供超时检测
    event_bus_publish(EVT_NET_STATUS_WIFI_CONN, 0);
    snprintf(buf, sizeof(buf), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    BSP_ESP8266_SendString(buf);
}

static void on_poll_wifi(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
        if (g_fsm_paused) return; 
    #endif
    Net_context_t* ctx = (Net_context_t*)arg;
    
    // 非阻塞检测：15秒内未收到 "WIFI GOT IP"，判为超时，抛出 ERROR 事件
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(15000)) {
        SYS_LOG("NET", "WiFi Connection Timeout! Retrying...\n");
        fsm_push_event(fsm, EVT_ERROR, 0);
    }
}

static void on_enter_mqtt(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->init_step = 0;
    ctx->last_tx_tick = FSM_GET_TICK(); // 重置时间基准
    event_bus_publish(EVT_NET_STATUS_MQTT_CONN, 0);
}
static void on_poll_mqtt(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
				if (g_fsm_paused) return; 
		#endif
    
    Net_context_t* ctx = (Net_context_t*)arg;
    uint32_t current_tick = FSM_GET_TICK();
    char buf[256];

    // 步骤 0：WIFI 连上后，严格等待 5000ms 发送 USERCFG
    if (ctx->init_step == 0 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(3000))) {
        BSP_ESP8266_SendString("AT+MQTTCLEAN=0\r\n");
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    }
    // 步骤 1：发送用户配置
    else if (ctx->init_step == 1 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(2000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTUSERCFG=0,1,\"NULL\",\"%s\",\"%s\",0,0,\"\"\r\n", USERNAME, PASSWORD);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    // 步骤 2：发送客户端 ID
    else if (ctx->init_step == 2 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(2000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTCLIENTID=0,\"%s\"\r\n", CLIENTID);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    // 步骤 3：建立连接
    else if (ctx->init_step == 3 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(3000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTCONN=0,\"%s\",1883,1\r\n", DOMAINNAME);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    }
    // 步骤 4：订阅控制 Topic
    else if (ctx->init_step == 4 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(2000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTSUB=0,\"/sys/%s/%s/thing/service/property/set\",0\r\n", 
                 PRODUCTKEY, DEVICENAME);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    }
    // 步骤 5：延时后跳转到 ONLINE 状态
    else if (ctx->init_step == 5 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(1000))) {
        fsm_push_event(fsm, EVT_SELECT_2, 0); 
    }
		
		
		
		
}
static void on_enter_online(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->last_tx_tick = FSM_GET_TICK();
    event_bus_publish(EVT_NET_STATUS_ONLINE, 0);
}

static void on_poll_online(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
				if (g_fsm_paused) return; 
		#endif
    Net_context_t* ctx = (Net_context_t*)arg;
    char pub_buf[512]; 
    char json_buf[256];
		// 用于控制欧拉角上传频率
    static uint32_t last_euler_tx_tick = 1500;
		//(3秒一次)
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(3000)) {
        ctx->last_tx_tick = FSM_GET_TICK();
        snprintf(json_buf, sizeof(json_buf), 
         "{\\\"id\\\":\\\"1\\\"\\,\\\"version\\\":\\\"1.0\\\"\\,\\\"params\\\":{\\\"temp\\\":%.1f\\,\\\"hum\\\":%.1f\\,\\\"smoke_density\\\":%.1f\\,\\\"LightLux\\\":%.1f\\,\\\"ultrasound_distance\\\":%.2f}}",
         g_sensor_data.temp, g_sensor_data.hum, g_sensor_data.smoke_ppm, g_sensor_data.light_lux, g_sensor_data.distance/100.0);
                
        snprintf(pub_buf, sizeof(pub_buf),
                 "AT+MQTTPUB=0,\"/sys/%s/%s/thing/event/property/post\",\"%s\",0,0\r\n",
                 PRODUCTKEY, DEVICENAME, json_buf);
        BSP_ESP8266_SendString(pub_buf);
    }
		
		// 2. 欧拉角高频上报 (如果开启且在线)
    if (ctx->euler_report_en && BSP_MPU6050_IsWorking()) {
        if (FSM_GET_TICK() - last_euler_tx_tick > FSM_MS_TO_TICKS(3000)) {
            last_euler_tx_tick = FSM_GET_TICK();
            
            // 直接读取 fsm_motor 中更新的全局变量，不再访问硬件 FIFO
            snprintf(json_buf, sizeof(json_buf), 
                "{\\\"params\\\":{\\\"Euler_angle_Roll\\\":%.1f\\,\\\"Euler_angle_Yaw\\\":%.1f\\,\\\"Euler_angle_Pitch\\\":%.1f}\\,\\\"version\\\":\\\"1.0.0\\\"}",
                g_imu_data.roll, g_imu_data.yaw, g_imu_data.pitch);
                
            snprintf(pub_buf, sizeof(pub_buf),
                "AT+MQTTPUB=0,\"/sys/%s/%s/thing/event/property/post\",\"%s\",0,0\r\n",
                PRODUCTKEY, DEVICENAME, json_buf);
            BSP_ESP8266_SendString(pub_buf);
        }
    }
}
static void on_enter_error(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->last_tx_tick = FSM_GET_TICK();
    event_bus_publish(EVT_NET_STATUS_ERROR, 0); // 通知主系统：网络寄了
}

static void on_poll_error(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    // 错误状态下休息 5 秒，然后触发超时事件，重新从 INIT 开始 AT+RST
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(5000)) {
        fsm_push_event(fsm, EVT_TIMEOUT, 0);
    }
}
// 增加转换动作：处理开关事件
static void action_euler_switch(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->euler_report_en = fsm->current_param;
}


// --- FSM 表 ---
static const fsm_state_desc_t net_states[] = {
    { STATE_NET_INIT,      on_enter_init,   NULL, on_poll_init },
    { STATE_NET_WIFI_CONN, on_enter_wifi,   NULL, on_poll_wifi }, 
    { STATE_NET_MQTT_CONN, on_enter_mqtt,   NULL, on_poll_mqtt },
    { STATE_NET_ONLINE,    on_enter_online, NULL, on_poll_online },
    { STATE_NET_ERROR,     on_enter_error,  NULL, on_poll_error },
};
// 更新转换表，确保所有错误先去 ERROR 状态，再由超时触发重启
static const fsm_transition_t net_trans[] = {
    { STATE_NET_INIT,      EVT_INIT_DONE, STATE_NET_WIFI_CONN, NULL, NULL },
    { STATE_NET_WIFI_CONN, EVT_SELECT_1,  STATE_NET_MQTT_CONN, NULL, NULL }, 
    
    // 【修改】任何阶段发生 ERROR，先去 ERROR 状态通报
    { STATE_NET_WIFI_CONN, EVT_ERROR,     STATE_NET_ERROR,     NULL, NULL },
    { STATE_NET_MQTT_CONN, EVT_ERROR,     STATE_NET_ERROR,     NULL, NULL },
    { STATE_NET_ONLINE,    EVT_ERROR,     STATE_NET_ERROR,     NULL, NULL },
    
    // 【新增】ERROR 状态休息 5 秒后，重新从 INIT 开始复位
    { STATE_NET_ERROR,     EVT_TIMEOUT,   STATE_NET_INIT,      NULL, NULL },
    
    { STATE_NET_MQTT_CONN, EVT_SELECT_2,  STATE_NET_ONLINE,    NULL, NULL },
    { STATE_NET_ONLINE, EVT_NET_EULER_OPEN, STATE_NET_ONLINE, NULL, action_euler_switch },
    { STATE_NET_ONLINE, EVT_NET_EULER_CLOSE, STATE_NET_ONLINE, NULL, action_euler_switch }, 
};
void Network_FSM_Setup(fsm_t* fsm) {
    if (!fsm) return;

    // 1. 初始化网络上下文 (Context)
    // 确保内部步骤归零，并且初始化时间戳以供后续 on_poll_online 等轮询函数使用
    net_ctx.init_step = 0;
    net_ctx.last_tx_tick = FSM_GET_TICK();
	
		// 确保开机绝对不会私自上传
    net_ctx.euler_report_en = 0;
    // 2. 初始化 FSM 核心
    // 绑定静态事件队列、状态转换表(net_trans)、初始状态及上下文参数
    fsm_init(fsm, 
             net_evt_buffer, 16, 
             net_trans, sizeof(net_trans)/sizeof(fsm_transition_t), 
             STATE_NET_INIT, 
             &net_ctx);

    // 3. 注册状态回调 (Enter, Poll, Exit)
    fsm_set_state_callbacks(fsm, 
                            net_states, 
                            sizeof(net_states)/sizeof(fsm_state_desc_t));

    // 4. 订阅总线事件 (核心规则：转换表里依赖什么外部事件，这里就订阅什么)
    event_bus_subscribe(fsm, EVT_INIT_DONE); // 订阅: AT基础指令配置完成事件
    event_bus_subscribe(fsm, EVT_SELECT_1);  // 订阅: WiFi 获取 IP 成功事件
    event_bus_subscribe(fsm, EVT_SELECT_2);  // 订阅: MQTT 鉴权及连接成功事件
    event_bus_subscribe(fsm, EVT_ERROR);     // 订阅: 系统级网络错误异常事件
		event_bus_subscribe(fsm, EVT_NET_EULER_OPEN);//  订阅欧拉角开事件
    event_bus_subscribe(fsm, EVT_NET_EULER_CLOSE);//  订阅欧拉角关事件
    // 5. 手动触发 INIT 状态的 Enter 回调，启动第一个 AT 动作
    on_enter_init(fsm, &net_ctx);
}

/* ================= 网络通信任务 ================= */
void Network_FSM_Task(void *pvParameters) {
    fsm_t fsm;
    Network_FSM_Setup(&fsm);
    uint16_t net_rx_len;
		#if ENABLE_DEBUG_PRINT
				uint16_t debug_rx_len;
		#endif
    
		while(1) {
			#if ENABLE_DEBUG_PRINT
					// 1. 非阻塞检测 PC 调试指令 (优先级高于网络解析)
					if (xQueueReceive(xDebugRxQueue, &debug_rx_len, 0) == pdPASS) {
							g_fsm_paused = 1; // 收到调试指令，激活拦截机制，暂停 FSM 自动轮询
							char* dbg_str = (char*)g_debug_rx_buf;
							
							// 将 PC 指令透传给 ESP8266。该函数已包含 [STM32 -> WIFI] 的流向打印。
							BSP_ESP8266_SendString(dbg_str);
					}
			#endif
        // 2. 阻塞等待 ESP8266 网络数据
        if (xQueueReceive(xNetRxQueue, &net_rx_len, pdMS_TO_TICKS(50)) == pdPASS) {
            char* rx_str = (char*)g_esp8266_rx_buf;
            
            // 打印带前缀的数据。此时处于任务级上下文中，符合 RTOS 规范。
            printf("[WIFI -> STM32]: %s", rx_str); 
            
            if (fsm.current_state == STATE_NET_WIFI_CONN && strstr(rx_str, "WIFI GOT IP")) {
                fsm_push_event(&fsm, EVT_SELECT_1, 0);
            }
            if (fsm.current_state == STATE_NET_MQTT_CONN && strstr(rx_str, "OK")) {
                if (net_ctx.init_step >= 5) fsm_push_event(&fsm, EVT_SELECT_2, 0);
            }
            if (fsm.current_state == STATE_NET_ONLINE) {
                parse_aliyun_payload(rx_str);
            }
        }
        
        // 3. 驱动状态机运转
        fsm_run(&fsm);
    }
}
