#include "fsm_network.h"
#include "event_bus.h"
#include "sys_events.h"
#include "bsp_esp8266.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "sys_config.h"
#include <string.h>
#include <stdio.h>
#include "bsp_debug_uart.h"
#include "sys_config.h"
#include "bsp_mpu6050.h"
#include "fsm_motor.h"
#include "fsm_sensor.h"
#include "state_repo.h"  // 中央状态仓库（层2 状态数据层）
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
// ===== Transparent-mode native MQTT 3.1.1 client (QoS0, no TLS) =====
// Used only when ESP8266_TRANSPARENT_MODE=1 / g_esp_mode=1. In AT mode(=0) the ESP8266 MQTT AT
// firmware is used, so this block is inert. After entering transparent mode the ESP8266 becomes a raw
// pipe and the MCU must speak MQTT itself. NOTE: Aliyun IoT requires an HMAC-SHA1 signed password;
// this project ships no crypto, so PASSWORD is passed verbatim -- sign it before real deployment.
#if ESP8266_TRANSPARENT_MODE
#define MQTT_KEEPALIVE_S   60
static uint8_t  g_mqtt_tx[ESP8266_RX_MAX];
static uint8_t  g_mqtt_rx_acc[ESP8266_RX_MAX];
static uint16_t g_mqtt_rx_acc_len = 0;
static volatile uint8_t g_mqtt_conn_ack    = 0;
static volatile uint8_t g_mqtt_sub_ack     = 0;
static volatile uint8_t g_mqtt_active      = 0;
static volatile uint8_t g_mqtt_handshake_ok= 0;
static volatile uint8_t g_mqtt_prompt_gt   = 0;
static uint32_t g_mqtt_last_ping_tick      = 0;

static void mqtt_put_u16(uint8_t** p, uint16_t v){ *((*p)++) = (uint8_t)(v>>8); *((*p)++) = (uint8_t)(v&0xFF); }
static void mqtt_put_str(uint8_t** p, const char* s){ uint16_t l=(uint16_t)strlen(s); mqtt_put_u16(p,l); while(*s){ *((*p)++) = (uint8_t)(*s++); } }
static uint8_t mqtt_enc_remlen(uint8_t* dst, uint32_t len){
    uint8_t n=0;
    do { uint8_t enc = (uint8_t)(len % 128); len /= 128; if(len) enc |= 0x80; dst[n++] = enc; } while(len>0);
    return n;
}
static uint16_t mqtt_wrap(uint8_t type, const uint8_t* body, uint32_t body_len, uint8_t* out){
    uint8_t* o = out;
    *o++ = type;
    uint8_t tn = mqtt_enc_remlen(o, body_len);
    o += tn;
    for (uint32_t i=0;i<body_len;i++) *o++ = body[i];
    return (uint16_t)(o-out);
}

static uint16_t Net_MqttBuildConnect(uint8_t* out){
    static uint8_t body[ESP8266_RX_MAX];
    uint8_t* p = body;
    mqtt_put_str(&p, "MQTT");
    *p++ = 4;
    *p++ = 0xC2;
    mqtt_put_u16(&p, MQTT_KEEPALIVE_S);
    mqtt_put_str(&p, CLIENTID);
    mqtt_put_str(&p, USERNAME);
    mqtt_put_str(&p, PASSWORD);
    return mqtt_wrap(0x10, body, (uint32_t)(p-body), out);
}
static uint16_t Net_MqttBuildPublish(uint8_t* out, const char* topic, const char* payload){
    static uint8_t body[ESP8266_RX_MAX];
    uint8_t* p = body;
    mqtt_put_str(&p, topic);
    const char* q = payload;
    while (*q) *p++ = (uint8_t)(*q++);
    return mqtt_wrap(0x30, body, (uint32_t)(p-body), out);
}
static uint16_t Net_MqttBuildSubscribe(uint8_t* out, uint16_t pkt_id, const char* topic){
    static uint8_t body[ESP8266_RX_MAX];
    uint8_t* p = body;
    mqtt_put_u16(&p, pkt_id);
    mqtt_put_str(&p, topic);
    *p++ = 0;
    return mqtt_wrap(0x82, body, (uint32_t)(p-body), out);
}

// Accumulate inbound bytes, extract complete packets; PUBLISH payload (pure JSON) -> parse_aliyun_payload.
static void Net_MqttParse(const uint8_t* chunk, uint16_t chunk_len){
    for (uint16_t i=0;i<chunk_len && g_mqtt_rx_acc_len < ESP8266_RX_MAX; i++)
        g_mqtt_rx_acc[g_mqtt_rx_acc_len++] = chunk[i];

    uint16_t pos = 0;
    while (g_mqtt_rx_acc_len - pos >= 2) {
        uint8_t  b0    = g_mqtt_rx_acc[pos];
        uint8_t  type  = b0 & 0xF0;
        uint32_t rem   = 0;
        uint8_t  L     = 0;
        uint8_t  shift = 0;
        int      ok    = 1;
        for (int k=1; ; k++) {
            if (pos + k >= g_mqtt_rx_acc_len) { ok=0; break; }
            uint8_t enc = g_mqtt_rx_acc[pos+k];
            rem |= (uint32_t)(enc & 0x7F) << shift;
            shift += 7; L++;
            if (!(enc & 0x80)) break;
            if (k >= 4) { ok=0; break; }
        }
        if (!ok) break;
        uint32_t total = 1 + L + rem;
        if (g_mqtt_rx_acc_len - pos < total) break;
        if (type == 0x30) {
            uint16_t pp   = pos + 1 + L;
            uint16_t tlen = (uint16_t)((g_mqtt_rx_acc[pp]<<8) | g_mqtt_rx_acc[pp+1]); pp += 2;
            const uint8_t* topic = &g_mqtt_rx_acc[pp]; (void)topic; pp += tlen;
            static char pl[ESP8266_RX_MAX];
            uint16_t plen = (uint16_t)(total - pp);
            uint16_t n = (plen < ESP8266_RX_MAX-1) ? plen : (ESP8266_RX_MAX-1);
            for (uint16_t i=0;i<n;i++) pl[i] = (char)g_mqtt_rx_acc[pp+i];
            pl[n] = '\0';
            parse_aliyun_payload(pl);
        } else if (b0 == 0x20) { g_mqtt_conn_ack = 1;
        } else if (b0 == 0x90) { g_mqtt_sub_ack  = 1;
        }
        pos += total;
    }
    if (pos > 0) {
        uint16_t keep = g_mqtt_rx_acc_len - pos;
        for (uint16_t i=0;i<keep;i++) g_mqtt_rx_acc[i] = g_mqtt_rx_acc[pos+i];
        g_mqtt_rx_acc_len = keep;
    }
}

// Transparent handshake state machine: CIPSTART -> CIPMODE=1 -> CIPSEND -> CONNECT -> SUBSCRIBE -> ONLINE
static void Net_MqttPollTransparent(fsm_t* fsm, Net_context_t* ctx, uint32_t now){
    switch (ctx->init_step) {
        case 0:
            BSP_ESP8266_SendString("AT+CIPSTART=\"TCP\",\"" DOMAINNAME "\",1883\r\n");
            ctx->last_tx_tick = now; ctx->init_step = 1; break;
        case 1:
            if (now - ctx->last_tx_tick >= FSM_MS_TO_TICKS(8000)) { fsm_push_event(fsm, EVT_ERROR,0); break; }
            if (g_mqtt_handshake_ok) { g_mqtt_handshake_ok=0; BSP_ESP8266_SendString("AT+CIPMODE=1\r\n");
                ctx->last_tx_tick = now; ctx->init_step = 2; }
            break;
        case 2:
            if (now - ctx->last_tx_tick >= FSM_MS_TO_TICKS(3000)) { fsm_push_event(fsm,EVT_ERROR,0); break; }
            if (g_mqtt_handshake_ok) { g_mqtt_handshake_ok=0; BSP_ESP8266_SendString("AT+CIPSEND\r\n");
                ctx->last_tx_tick = now; ctx->init_step = 3; }
            break;
        case 3:
            if (now - ctx->last_tx_tick >= FSM_MS_TO_TICKS(3000)) { fsm_push_event(fsm,EVT_ERROR,0); break; }
            if (g_mqtt_prompt_gt) { g_mqtt_prompt_gt=0; BSP_ESP8266_SwitchMode(1); g_mqtt_active=1;
                uint16_t n = Net_MqttBuildConnect(g_mqtt_tx); BSP_ESP8266_SendRaw(g_mqtt_tx, n);
                g_mqtt_conn_ack=0; g_mqtt_last_ping_tick=now; ctx->last_tx_tick=now; ctx->init_step=4; }
            break;
        case 4:
            if (now - ctx->last_tx_tick >= FSM_MS_TO_TICKS(5000)) { fsm_push_event(fsm,EVT_ERROR,0); break; }
            if (g_mqtt_conn_ack) { g_mqtt_conn_ack=0;
                uint16_t n = Net_MqttBuildSubscribe(g_mqtt_tx, 1, "/sys/" PRODUCTKEY "/" DEVICENAME "/thing/service/property/set");
                BSP_ESP8266_SendRaw(g_mqtt_tx, n);
                g_mqtt_sub_ack=0; ctx->last_tx_tick=now; ctx->init_step=5; }
            break;
        case 5:
            if (now - ctx->last_tx_tick >= FSM_MS_TO_TICKS(5000)) { fsm_push_event(fsm,EVT_ERROR,0); break; }
            if (g_mqtt_sub_ack) { g_mqtt_sub_ack=0; fsm_push_event(fsm, EVT_SELECT_2, 0); }
            break;
        default: break;
    }
}
#endif /* ESP8266_TRANSPARENT_MODE */

static void on_enter_init(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->init_step = 0;
    ctx->last_tx_tick = FSM_GET_TICK();
    g_net_link_lost = 1;   // 启动时未连网：链路丢失，motor 任务保持安全态直到上线
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
    // 透传模式：走原生 MQTT 握手状态机（AT+CIPSTART→CIPMODE=1→CIPSEND→CONNECT→SUBSCRIBE）
#if ESP8266_TRANSPARENT_MODE
    if (g_esp_mode) { Net_MqttPollTransparent(fsm, ctx, current_tick); return; }
#endif
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
/* P0-① 失联即停：网络心跳超时阈值。在线态超过此时长未成功上行 MQTT（last_tx_tick 未刷新），
 * 判定遥控/云端心跳丢失，置 g_net_link_lost 供 motor 任务进安全态。 */
#define NET_HEARTBEAT_TIMEOUT_MS  10000

static void on_enter_online(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->last_tx_tick = FSM_GET_TICK();
    g_net_link_lost = 0;   // 进入在线态：链路恢复
    event_bus_publish(EVT_NET_STATUS_ONLINE, 0);
}

static void on_poll_online(fsm_t* fsm, void* arg) {
    #if ENABLE_DEBUG_PRINT
				if (g_fsm_paused) return; 
		#endif
    Net_context_t* ctx = (Net_context_t*)arg;
    /* P0-① 失联检测：在线态但超过心跳阈值未成功上行 → 判失联，motor 任务进安全态 */
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(NET_HEARTBEAT_TIMEOUT_MS)) {
        g_net_link_lost = 1;
    }
    char pub_buf[512]; 
    char json_buf[256];
		// ���ڿ���ŷ�����ϴ�Ƶ��
    static uint32_t last_euler_tx_tick = 1500;
		//(3��һ��)
    if (FSM_GET_TICK() - ctx->last_tx_tick > FSM_MS_TO_TICKS(3000)) {
        ctx->last_tx_tick = FSM_GET_TICK();
        // 在互斥量保护下读取跨任务共享的传感器数据（防 float 读写撕裂）
        float t = 0, hu = 0, sm = 0, lu = 0, di = 0;
        SensorData_t sdata_net;
        if (state_read_if_new(ST_SENSOR_DATA, &sdata_net, &ctx->last_sensor_seq)) {
            t  = sdata_net.temp;
            hu = sdata_net.hum;
            sm = sdata_net.smoke_ppm;
            lu = sdata_net.light_lux;
            di = sdata_net.distance;
        }
        snprintf(json_buf, sizeof(json_buf),
         "{\\\"id\\\":\\\"1\\\"\\,\\\"version\\\":\\\"1.0\\\"\\,\\\"params\\\":{\\\"temp\\\":%.1f\\,\\\"hum\\\":%.1f\\,\\\"smoke_density\\\":%.1f\\,\\\"LightLux\\\":%.1f\\,\\\"ultrasound_distance\\\":%.2f}}",
         t, hu, sm, lu, di/100.0);

#if ESP8266_TRANSPARENT_MODE
        if (g_esp_mode) {
            // 透传：原生 MQTT PUBLISH(QoS0) + keepalive 维持
            uint16_t n = Net_MqttBuildPublish(g_mqtt_tx,
                "/sys/" PRODUCTKEY "/" DEVICENAME "/thing/event/property/post", json_buf);
            BSP_ESP8266_SendRaw(g_mqtt_tx, n);
            ctx->last_tx_tick = FSM_GET_TICK();
            if (FSM_GET_TICK() - g_mqtt_last_ping_tick > FSM_MS_TO_TICKS((MQTT_KEEPALIVE_S*1000)/2)) {
                uint8_t ping[2] = {0xC0, 0x00};
                BSP_ESP8266_SendRaw(ping, 2);
                g_mqtt_last_ping_tick = FSM_GET_TICK();
            }
        } else
#endif
        {
            snprintf(pub_buf, sizeof(pub_buf),
                     "AT+MQTTPUB=0,\"/sys/%s/%s/thing/event/property/post\",\"%s\",0,0\r\n",
                     PRODUCTKEY, DEVICENAME, json_buf);
            BSP_ESP8266_SendString(pub_buf);
        }
    }
		
		// 2. ŷ���Ǹ�Ƶ�ϱ� (�������������)
    if (ctx->euler_report_en && BSP_MPU6050_IsWorking()) {
        if (FSM_GET_TICK() - last_euler_tx_tick > FSM_MS_TO_TICKS(3000)) {
            last_euler_tx_tick = FSM_GET_TICK();
            // 从中央状态仓库读取最新 IMU 数据（Motor 单写者写入，无锁 + seq 保证完整）
            MPU6050_Data_t imu_net;
            float er = 0.0f, ey = 0.0f, ep = 0.0f;
            if (state_read_if_new(ST_IMU_DATA, &imu_net, &ctx->last_imu_seq)) {
                er = imu_net.roll; ey = imu_net.yaw; ep = imu_net.pitch;
            }
            
            // ֱ�Ӷ�ȡ fsm_motor �и��µ�ȫ�ֱ��������ٷ���Ӳ�� FIFO
            snprintf(json_buf, sizeof(json_buf), 
                "{\\\"params\\\":{\\\"Euler_angle_Roll\\\":%.1f\\,\\\"Euler_angle_Yaw\\\":%.1f\\,\\\"Euler_angle_Pitch\\\":%.1f}\\,\\\"version\\\":\\\"1.0.0\\\"}",
                er, ey, ep);
                
#if ESP8266_TRANSPARENT_MODE
            if (g_esp_mode) {
                uint16_t n = Net_MqttBuildPublish(g_mqtt_tx,
                    "/sys/" PRODUCTKEY "/" DEVICENAME "/thing/event/property/post", json_buf);
                BSP_ESP8266_SendRaw(g_mqtt_tx, n);
            } else
#endif
            {
                snprintf(pub_buf, sizeof(pub_buf),
                    "AT+MQTTPUB=0,\"/sys/%s/%s/thing/event/property/post\",\"%s\",0,0\r\n",
                    PRODUCTKEY, DEVICENAME, json_buf);
                BSP_ESP8266_SendString(pub_buf);
            }
        }
    }
}
static void on_enter_error(fsm_t* fsm, void* arg) {
    Net_context_t* ctx = (Net_context_t*)arg;
    ctx->last_tx_tick = FSM_GET_TICK();
    g_net_link_lost = 1;
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

		while(1) {
        // 2. �����ȴ� ESP8266 ��������
        uint32_t notify_val = 0;
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notify_val, pdMS_TO_TICKS(50)) == pdPASS) {
            #if ENABLE_DEBUG_PRINT
            // 调试串口收到一帧：从数据池取块 → 透传到 ESP8266 → 归还空闲池（N=2 双队列资源池化）
            debug_buf_t* p_dbg = NULL;
            if (xQueueReceive(xDebugDataQueue, &p_dbg, 0) == pdPASS && p_dbg != NULL) {
                g_fsm_paused = 1;
                BSP_ESP8266_SendString((char*)p_dbg->data);
                p_dbg->len = 0;
                xQueueSend(xDebugFreeQueue, &p_dbg, 0);  // 归还空闲池，闭环
                // 背压解除：若 DMA 因 FreeQueue 空而暂停，任务归还后恢复 DMA
                if (g_dma_suspended) {
                    BSP_DebugUART_ResumeDMA();
                }
            }
            #endif
#if ESP8266_TRANSPARENT_MODE
                        // ESP8266 数据接收：按 g_esp_mode 分支（透传=环形 drain，AT=双队列取块）
            if (g_esp_mode) {
                // 透传模式（Circular + 环形）：从环形缓冲拷贝可用字节（纯裸流，无 +IPD 前缀）
                static char net_line_buf[ESP8266_RX_MAX + 1];
                uint16_t n = BSP_ESP8266_RxDrain(net_line_buf, sizeof(net_line_buf));
                if (n > 0) {
                    char* rx_str = net_line_buf;
                    printf("[WIFI -> STM32]: %s", rx_str);

                    if (fsm.current_state == STATE_NET_WIFI_CONN && strstr(rx_str, "WIFI GOT IP")) {
                        fsm_push_event(&fsm, EVT_SELECT_1, 0);
                    }
                    if (fsm.current_state == STATE_NET_MQTT_CONN) {
                        if (strstr(rx_str, "OK")) g_mqtt_handshake_ok = 1;
                        if (strstr(rx_str, ">"))  g_mqtt_prompt_gt   = 1;
                        if (net_ctx.init_step >= 5) fsm_push_event(&fsm, EVT_SELECT_2, 0);
                    }
                    if (fsm.current_state == STATE_NET_ONLINE) {
                        Net_MqttParse((uint8_t*)rx_str, n);
                    }
                }
            } else
#endif
            {
                // AT 模式（Normal + 双队列）：取一块完整 MQTT 封装包 → 解析 → 归还空闲池（N=4）
                esp8266_buf_t* p_esp = NULL;
                if (xQueueReceive(xEspDataQueue, &p_esp, 0) == pdPASS && p_esp != NULL) {
                    char* rx_str = (char*)p_esp->data;
                    // 确保字符串安全终止（块内已按实际接收长度写入，但加冗余保护）
                    if (p_esp->len < ESP8266_RX_MAX) p_esp->data[p_esp->len] = '\0';

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

                    // 归还空闲池，闭环；若 DMA 曾因 FreeQueue 空暂停，则恢复
                    p_esp->len = 0;
                    xQueueSend(xEspFreeQueue, &p_esp, 0);
                    if (g_esp_dma_suspended) {
                        BSP_ESP8266_ResumeDMA();
                    }
                }
            }
        }
        
        // 3. ����״̬����ת
        fsm_run(&fsm);
    }
}
