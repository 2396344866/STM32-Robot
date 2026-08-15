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
// 1. �������崦
#if ENABLE_DEBUG_PRINT
static uint8_t g_fsm_paused = 0;
#endif
#define WIFI_SSID       "WIFI_SSID"
#define WIFI_PWD        "WIFI_PWD"
#define USERNAME        "USERNAME"
#define PASSWORD        "PASSWORD"
#define CLIENTID        "CLIENTID"
#define DOMAINNAME      "DOMAINNAME"
#define DEVICENAME      "DEVICENAME"
#define PRODUCTKEY      "PRODUCTKEY" 


static fsm_event_t net_evt_buffer[16];
static Net_context_t net_ctx;
//GlobalSensorData_t g_SensorData;
#define APP_SOURCE 2
#define ENCODE_PARAM(source, evt) (((uint32_t)(source) << 16) | (evt))

// --- JSON ���� (��һ��ԭ�����ײ��Ӵ�ƥ��) ---
static void parse_aliyun_payload(const char* payload) {
    uint16_t target_evt = EVT_NONE;

    // 1. ƥ�䶯��ָ�� (value:1)
    if (strstr(payload, "\"value\":1")) {
        if (strstr(payload, "\"move_on\""))             target_evt = EVT_MOTOR_FORWARD;
        else if (strstr(payload, "\"move_back\""))      target_evt = EVT_MOTOR_BACKWARD;
        else if (strstr(payload, "\"move_left\""))      target_evt = EVT_MOTOR_LEFT;
        else if (strstr(payload, "\"move_right\""))     target_evt = EVT_MOTOR_RIGHT;
        else if (strstr(payload, "\"move_left_rotate\"")) target_evt = EVT_MOTOR_ROT_L;
        else if (strstr(payload, "\"move_right_rotate\""))target_evt = EVT_MOTOR_ROT_R;
        else if (strstr(payload, "\"move_stop\""))      target_evt = EVT_MOTOR_STOP;
    } 
    // 2. ƥ�临λ/ָֹͣ�� (value:0)
    else if (strstr(payload, "\"value\":0")) {
        target_evt = EVT_MOTOR_STOP;
    }

    // 3. ��������������¼�
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


// --- ״̬�ص� ---
static void on_enter_init(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->init_step = 0;
    ctx->last_tx_tick = FSM_GET_TICK(); // ��ʼ��ʱ���׼
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
    ctx->last_tx_tick = FSM_GET_TICK(); // ��¼��ʼʱ����Թ���ʱ���
    event_bus_publish(EVT_NET_STATUS_WIFI_CONN, 0);
    snprintf(buf, sizeof(buf), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    BSP_ESP8266_SendString(buf);
}

static void on_poll_wifi(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
        if (g_fsm_paused) return; 
    #endif
    Net_context_t* ctx = (Net_context_t*)arg;
    
    // ��������⣺15����δ�յ� "WIFI GOT IP"����Ϊ��ʱ���׳� ERROR �¼�
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(15000)) {
        SYS_LOG("NET", "WiFi Connection Timeout! Retrying...\n");
        fsm_push_event(fsm, EVT_ERROR, 0);
    }
}

static void on_enter_mqtt(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->init_step = 0;
    ctx->last_tx_tick = FSM_GET_TICK(); // ����ʱ���׼
    event_bus_publish(EVT_NET_STATUS_MQTT_CONN, 0);
}
static void on_poll_mqtt(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
				if (g_fsm_paused) return; 
		#endif
    
    Net_context_t* ctx = (Net_context_t*)arg;
    uint32_t current_tick = FSM_GET_TICK();
    char buf[256];

    // ���� 0��WIFI ���Ϻ��ϸ�ȴ� 5000ms ���� USERCFG
    if (ctx->init_step == 0 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(3000))) {
        BSP_ESP8266_SendString("AT+MQTTCLEAN=0\r\n");
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    }
    // ���� 1�������û�����
    else if (ctx->init_step == 1 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(2000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTUSERCFG=0,1,\"NULL\",\"%s\",\"%s\",0,0,\"\"\r\n", USERNAME, PASSWORD);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    // ���� 2�����Ϳͻ��� ID
    else if (ctx->init_step == 2 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(2000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTCLIENTID=0,\"%s\"\r\n", CLIENTID);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    } 
    // ���� 3����������
    else if (ctx->init_step == 3 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(3000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTCONN=0,\"%s\",1883,1\r\n", DOMAINNAME);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    }
    // ���� 4�����Ŀ��� Topic
    else if (ctx->init_step == 4 && (current_tick - ctx->last_tx_tick >= FSM_MS_TO_TICKS(2000))) {
        snprintf(buf, sizeof(buf), "AT+MQTTSUB=0,\"/sys/%s/%s/thing/service/property/set\",0\r\n", 
                 PRODUCTKEY, DEVICENAME);
        BSP_ESP8266_SendString(buf);
        ctx->last_tx_tick = current_tick;
        ctx->init_step++;
    }
    // ���� 5����ʱ����ת�� ONLINE ״̬
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
		// ���ڿ���ŷ�����ϴ�Ƶ��
    static uint32_t last_euler_tx_tick = 1500;
		//(3��һ��)
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
		
		// 2. ŷ���Ǹ�Ƶ�ϱ� (�������������)
    if (ctx->euler_report_en && BSP_MPU6050_IsWorking()) {
        if (FSM_GET_TICK() - last_euler_tx_tick > FSM_MS_TO_TICKS(3000)) {
            last_euler_tx_tick = FSM_GET_TICK();
            
            // ֱ�Ӷ�ȡ fsm_motor �и��µ�ȫ�ֱ��������ٷ���Ӳ�� FIFO
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
    event_bus_publish(EVT_NET_STATUS_ERROR, 0); // ֪ͨ��ϵͳ���������
}

static void on_poll_error(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    // ����״̬����Ϣ 5 �룬Ȼ�󴥷���ʱ�¼������´� INIT ��ʼ AT+RST
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(5000)) {
        fsm_push_event(fsm, EVT_TIMEOUT, 0);
    }
}
// ����ת�����������������¼�
static void action_euler_switch(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->euler_report_en = fsm->current_param;
}


// --- FSM �� ---
static const fsm_state_desc_t net_states[] = {
    { STATE_NET_INIT,      on_enter_init,   NULL, on_poll_init },
    { STATE_NET_WIFI_CONN, on_enter_wifi,   NULL, on_poll_wifi }, 
    { STATE_NET_MQTT_CONN, on_enter_mqtt,   NULL, on_poll_mqtt },
    { STATE_NET_ONLINE,    on_enter_online, NULL, on_poll_online },
    { STATE_NET_ERROR,     on_enter_error,  NULL, on_poll_error },
};
// ����ת������ȷ�����д�����ȥ ERROR ״̬�����ɳ�ʱ��������
static const fsm_transition_t net_trans[] = {
    { STATE_NET_INIT,      EVT_INIT_DONE, STATE_NET_WIFI_CONN, NULL, NULL },
    { STATE_NET_WIFI_CONN, EVT_SELECT_1,  STATE_NET_MQTT_CONN, NULL, NULL }, 
    
    // ���޸ġ��κν׶η��� ERROR����ȥ ERROR ״̬ͨ��
    { STATE_NET_WIFI_CONN, EVT_ERROR,     STATE_NET_ERROR,     NULL, NULL },
    { STATE_NET_MQTT_CONN, EVT_ERROR,     STATE_NET_ERROR,     NULL, NULL },
    { STATE_NET_ONLINE,    EVT_ERROR,     STATE_NET_ERROR,     NULL, NULL },
    
    // ��������ERROR ״̬��Ϣ 5 ������´� INIT ��ʼ��λ
    { STATE_NET_ERROR,     EVT_TIMEOUT,   STATE_NET_INIT,      NULL, NULL },
    
    { STATE_NET_MQTT_CONN, EVT_SELECT_2,  STATE_NET_ONLINE,    NULL, NULL },
    { STATE_NET_ONLINE, EVT_NET_EULER_OPEN, STATE_NET_ONLINE, NULL, action_euler_switch },
    { STATE_NET_ONLINE, EVT_NET_EULER_CLOSE, STATE_NET_ONLINE, NULL, action_euler_switch }, 
};
void Network_FSM_Setup(fsm_t* fsm) {
    if (!fsm) return;

    // 1. ��ʼ������������ (Context)
    // ȷ���ڲ�������㣬���ҳ�ʼ��ʱ����Թ����� on_poll_online ����ѯ����ʹ��
    net_ctx.init_step = 0;
    net_ctx.last_tx_tick = FSM_GET_TICK();
	
		// ȷ���������Բ���˽���ϴ�
    net_ctx.euler_report_en = 0;
    // 2. ��ʼ�� FSM ����
    // �󶨾�̬�¼����С�״̬ת����(net_trans)����ʼ״̬�������Ĳ���
    fsm_init(fsm, 
             net_evt_buffer, 16, 
             net_trans, sizeof(net_trans)/sizeof(fsm_transition_t), 
             STATE_NET_INIT, 
             &net_ctx);

    // 3. ע��״̬�ص� (Enter, Poll, Exit)
    fsm_set_state_callbacks(fsm, 
                            net_states, 
                            sizeof(net_states)/sizeof(fsm_state_desc_t));

    // 4. ���������¼� (���Ĺ���ת����������ʲô�ⲿ�¼�������Ͷ���ʲô)
    event_bus_subscribe(fsm, EVT_INIT_DONE); // ����: AT����ָ����������¼�
    event_bus_subscribe(fsm, EVT_SELECT_1);  // ����: WiFi ��ȡ IP �ɹ��¼�
    event_bus_subscribe(fsm, EVT_SELECT_2);  // ����: MQTT ��Ȩ�����ӳɹ��¼�
    event_bus_subscribe(fsm, EVT_ERROR);     // ����: ϵͳ����������쳣�¼�
		event_bus_subscribe(fsm, EVT_NET_EULER_OPEN);//  ����ŷ���ǿ��¼�
    event_bus_subscribe(fsm, EVT_NET_EULER_CLOSE);//  ����ŷ���ǹ��¼�
    // 5. �ֶ����� INIT ״̬�� Enter �ص���������һ�� AT ����
    on_enter_init(fsm, &net_ctx);
}

/* ================= ����ͨ������ ================= */
void Network_FSM_Task(void *pvParameters) {
    fsm_t fsm;
    Network_FSM_Setup(&fsm);
		#if ENABLE_DEBUG_PRINT
				uint16_t debug_rx_len;
		#endif
    
		while(1) {
			#if ENABLE_DEBUG_PRINT
					// 1. ��������� PC ����ָ�� (���ȼ������������)
					if (xQueueReceive(xDebugRxQueue, &debug_rx_len, 0) == pdPASS) {
							g_fsm_paused = 1; // �յ�����ָ��������ػ��ƣ���ͣ FSM �Զ���ѯ
							char* dbg_str = (char*)g_debug_rx_buf;
							
							// �� PC ָ��͸���� ESP8266���ú����Ѱ��� [STM32 -> WIFI] �������ӡ��
							BSP_ESP8266_SendString(dbg_str);
					}
			#endif
        // 2. �����ȴ� ESP8266 ��������
        if (xSemaphoreTake(xNetRxSem, pdMS_TO_TICKS(50)) == pdPASS) {
            static char net_line_buf[ESP8266_RX_MAX + 1];
            uint16_t n = BSP_ESP8266_RxDrain(net_line_buf, sizeof(net_line_buf));
            char* rx_str = net_line_buf;
            
            // ��ӡ��ǰ׺�����ݡ���ʱ���������������У����� RTOS �淶��
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
        
        // 3. ����״̬����ת
        fsm_run(&fsm);
    }
}
