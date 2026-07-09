#include "fsm_main_system.h"
#include "event_bus.h"
#include <stdio.h>
#include "bsp_LED.h"
#include "bsp_OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsm_key.h"

#define Main_System_QUEUE_SIZE 16
#define LOCAL_SOURCE 1 
#define APP_SOURCE   2
#define DECODE_SOURCE(param) ((uint8_t)((param) >> 16))
#define DECODE_EVT(param)    ((uint16_t)((param) & 0xFFFF))
#define ENCODE_PARAM(source, evt) (((uint32_t)(source) << 16) | (evt))

static fsm_event_t Main_System_evt_buffer[Main_System_QUEUE_SIZE];
static Main_System_context_t Main_System_ctx;
static fsm_t g_Main_System_fsm;

typedef struct {
    const char* name;
    sys_event_t event_id;
} mode_item_t;

static const mode_item_t MOTOR_ITEMS[] = {
    { "Forward",  EVT_MOTOR_FORWARD },
    { "Backward", EVT_MOTOR_BACKWARD },
    { "Left",     EVT_MOTOR_LEFT },
    { "Right",    EVT_MOTOR_RIGHT },
    { "Rot L",    EVT_MOTOR_ROT_L },
    { "Rot R",    EVT_MOTOR_ROT_R }
};
#define MOTOR_ITEM_COUNT (sizeof(MOTOR_ITEMS) / sizeof(mode_item_t))

static void update_motor_display(Main_System_context_t* ctx) {
    BSP_OLED_Clear();
    BSP_OLED_ShowString(1, 1, "--- CONTROL ---");
    
    char buf[16];
    snprintf(buf, sizeof(buf), "Act: %s", MOTOR_ITEMS[ctx->m1_menu_index].name);
    BSP_OLED_ShowString(2, 1, buf);

    // 区分控制源显示
    if (ctx->run_source == LOCAL_SOURCE) {
        BSP_OLED_ShowString(3, 1, "Status: RUNNING");
        BSP_OLED_ShowString(4, 1, "K2: Stop");
    } 
    else if (ctx->run_source == APP_SOURCE) {
        BSP_OLED_ShowString(3, 1, "Status: APP RUN");
        BSP_OLED_ShowString(4, 1, "APP/K2 to Stop");
    } 
    else {
        BSP_OLED_ShowString(3, 1, "Status: READY");
        BSP_OLED_ShowString(4, 1, "K1:SEL K2:ACT");
    }
}

// 被动同步：统一接管所有来自总线的电机指令 (涵盖 APP 与本地按键)
static void action_sync_motor_state(fsm_t* fsm, void* arg) {
    Main_System_context_t* ctx = (Main_System_context_t*)arg;
    uint8_t source = DECODE_SOURCE(fsm->current_param);
    uint16_t evt = DECODE_EVT(fsm->current_param);

    // 核心逻辑：判断电机目标状态
    if (evt == EVT_MOTOR_STOP || evt == EVT_NONE) {
        ctx->run_source = 0; 
        
        // 动作停止，重新激活待机定时器
        xTimerReset(ctx->xMenuTimer, 0); 
    } else {
        ctx->run_source = source ? source : APP_SOURCE; 
        
        // 动作运行，强制关闭待机定时器，禁止进入待机模式
        xTimerStop(ctx->xMenuTimer, 0);  
        
        for (int i = 0; i < MOTOR_ITEM_COUNT; i++) {
            if (MOTOR_ITEMS[i].event_id == evt) {
                ctx->m1_menu_index = i;
                break;
            }
        }
    }
    update_motor_display(ctx);
}
// K1 短按：切换动作
static void action_local_select(fsm_t* fsm, void* arg) {
    Main_System_context_t* ctx = (Main_System_context_t*)arg;
    if (ctx->run_source == 0) {
        ctx->m1_menu_index = (ctx->m1_menu_index + 1) % MOTOR_ITEM_COUNT;
        
        // 逻辑变更：用户操作，重置待机时间
        xTimerReset(ctx->xMenuTimer, 0); 
        update_motor_display(ctx);
    }
}

// K2 短按：启停翻转
static void action_local_toggle(fsm_t* fsm, void* arg) {
    Main_System_context_t* ctx = (Main_System_context_t*)arg;
    if (ctx->run_source != 0) {
        event_bus_publish(EVT_MOTOR_STOP, ENCODE_PARAM(LOCAL_SOURCE, EVT_MOTOR_STOP));
        ctx->run_source = 0;
        
        // 逻辑变更：电机停止，开始待机倒计时
        xTimerReset(ctx->xMenuTimer, 0); 
    } else {
        sys_event_t evt = MOTOR_ITEMS[ctx->m1_menu_index].event_id;
        event_bus_publish(evt, ENCODE_PARAM(LOCAL_SOURCE, evt));
        ctx->run_source = LOCAL_SOURCE;
        
        // 逻辑变更：电机运行，关闭待机定时器
        xTimerStop(ctx->xMenuTimer, 0); 
    }
    update_motor_display(ctx);
}
static void on_enter_init(fsm_t* fsm, void* arg) {
    fsm_push_event(fsm, EVT_INIT_DONE, 0);
}

static void on_enter_idle(fsm_t* fsm, void* arg) {
    BSP_OLED_Clear();
    BSP_OLED_ShowString(1, 1, "SYSTEM STANDBY");
    BSP_OLED_ShowString(3, 1, "Any Key Wakeup");
    BSP_LED1_Off();
}

static void on_enter_motor_ctrl(fsm_t* fsm, void* arg) {
    Main_System_context_t* ctx = (Main_System_context_t*)arg;
    update_motor_display(ctx);
    
    // 如果进入时电机是停止的，启动倒计时
    if (ctx->run_source == 0) {
        xTimerReset(ctx->xMenuTimer, 0);
    }
}
// 1. 在 on_enter_error_ui 中加一句打印，方便观察
static void on_enter_error_ui(fsm_t* fsm, void* arg) {
    Main_System_context_t* ctx = (Main_System_context_t*)arg;
    
    SYS_LOG("MAIN", "Network Error! UI Show Failed, Enter Standby in 2s...\n"); // 加入这行打印
    
    BSP_OLED_Clear();
    BSP_OLED_ShowString(2, 1, "Connect Failed!");
    BSP_OLED_ShowString(3, 1, "Enter Standby..");
    
    xTimerChangePeriod(ctx->xMenuTimer, FSM_MS_TO_TICKS(2000), 0);
    xTimerStart(ctx->xMenuTimer, 0);
}
static void on_enter_linking(fsm_t* fsm, void* arg) {
    BSP_OLED_Clear();
    BSP_OLED_ShowString(2, 1, "MQTT Linking...");
    BSP_OLED_ShowString(3, 1, "Please wait");
    SYS_LOG("MAIN", "Blocking inputs, waiting for MQTT...\n");
}
// 待机超时回调函数
static void vIdleTimeoutCallback(TimerHandle_t xTimer) {
    fsm_t* fsm = (fsm_t*)pvTimerGetTimerID(xTimer);
    fsm_push_event(fsm, EVT_TIMEOUT, 0);
}
static void on_enter_blocked(fsm_t* fsm, void* arg) {
    Main_System_context_t* ctx = (Main_System_context_t*)arg;
    BSP_OLED_Clear();
    BSP_OLED_ShowString(1, 1, "!!! WARNING !!!");
    BSP_OLED_ShowString(3, 1, "OBSTACLE DETECT");
    BSP_OLED_ShowString(4, 1, "MOTOR LOCKED");
    
    // 强制切断电机动力，并将控制源置 0
    event_bus_publish(EVT_MOTOR_STOP, ENCODE_PARAM(LOCAL_SOURCE, EVT_MOTOR_STOP));
    ctx->run_source = 0; 
    
    // 停止待机定时器
    xTimerStop(ctx->xMenuTimer, 0);
}

static const fsm_state_desc_t Main_System_state_descs[] = {
    { STATE_INIT,       on_enter_init,       NULL, NULL },
    { STATE_LINKING,    on_enter_linking,    NULL, NULL },
    { STATE_IDLE,       on_enter_idle,       NULL, NULL },
    { STATE_MOTOR_CTRL, on_enter_motor_ctrl, NULL, NULL },
		{ STATE_BLOCKED,    on_enter_blocked,    NULL, NULL }, // 注册避障状态
		{ STATE_ERROR,      on_enter_error_ui,   NULL, NULL }, // 激活 ERROR 状态
		
		
};
// 展开所有电机同步事件的宏，使代码整洁
#define TRANS_SYNC_MOTOR(state_from, state_to) \
    { state_from, EVT_MOTOR_FORWARD,  state_to, NULL, action_sync_motor_state }, \
    { state_from, EVT_MOTOR_BACKWARD, state_to, NULL, action_sync_motor_state }, \
    { state_from, EVT_MOTOR_LEFT,     state_to, NULL, action_sync_motor_state }, \
    { state_from, EVT_MOTOR_RIGHT,    state_to, NULL, action_sync_motor_state }, \
    { state_from, EVT_MOTOR_ROT_L,    state_to, NULL, action_sync_motor_state }, \
    { state_from, EVT_MOTOR_ROT_R,    state_to, NULL, action_sync_motor_state }, \
    { state_from, EVT_MOTOR_STOP,     state_to, NULL, action_sync_motor_state }

static const fsm_transition_t Main_System_transitions[] = {
    // 初始化上电直接进入控制界面
		{ STATE_INIT,       EVT_INIT_DONE,         STATE_LINKING,    NULL, NULL },
		// 2. 只有收到底层网络 FSM 发布的上线事件，才放行进入控制模式
    { STATE_LINKING,    EVT_NET_STATUS_ONLINE, STATE_MOTOR_CTRL, NULL, NULL },
    
		// 【新增】收到网络错误，进入UI报错界面
    { STATE_LINKING,    EVT_NET_STATUS_ERROR,  STATE_ERROR,      NULL, NULL },
    // 【新增】报错界面等待2秒超时后，自动退回待机
    { STATE_ERROR,      EVT_TIMEOUT,           STATE_IDLE,       NULL, NULL },
		
		// 待机模式逻辑 (任意键唤醒)
    { STATE_IDLE,       EVT_KEY1_SHORT_PRESS, STATE_MOTOR_CTRL, NULL, NULL },
    { STATE_IDLE,       EVT_KEY2_SHORT_PRESS, STATE_MOTOR_CTRL, NULL, NULL },
		{ STATE_MOTOR_CTRL, EVT_TIMEOUT, STATE_IDLE, NULL, NULL },
		// -------------- 避障抢占与解除逻辑 ---------------
    // 任何运行状态收到警告，立即切入 BLOCKED 状态
    { STATE_IDLE,       EVT_WARN_OBSTACLE, STATE_BLOCKED, NULL, NULL },
    { STATE_MOTOR_CTRL, EVT_WARN_OBSTACLE, STATE_BLOCKED, NULL, NULL },
    // 只有收到解除事件，才允许退回 IDLE。注意：在此状态下没有订阅按键和APP指令，实现物理隔离
    { STATE_BLOCKED,    EVT_OBSTACLE_CLEARED, STATE_IDLE, NULL, NULL },
		// ---+-----------------+---   ---+-----------------+---    ---+-----------------+---
    // 待机时收到 APP 指令，直接跳转控制界面并强制同步动作
    TRANS_SYNC_MOTOR(STATE_IDLE, STATE_MOTOR_CTRL),

    // 主控制逻辑
    { STATE_MOTOR_CTRL, EVT_KEY1_SHORT_PRESS, STATE_MOTOR_CTRL, NULL, action_local_select },
    { STATE_MOTOR_CTRL, EVT_KEY2_SHORT_PRESS, STATE_MOTOR_CTRL, NULL, action_local_toggle },
    { STATE_MOTOR_CTRL, EVT_KEY2_LONG_PRESS,  STATE_IDLE,       NULL, NULL }, // 长按待机
    // 控制界面内的状态同步（自流转）
    TRANS_SYNC_MOTOR(STATE_MOTOR_CTRL, STATE_MOTOR_CTRL),
		
};
void Main_System_fsm_setup(fsm_t* fsm) {
    if (!fsm) return;
		if (Main_System_ctx.xMenuTimer == NULL) {
        Main_System_ctx.xMenuTimer = xTimerCreate("MenuTmr", 
                                          FSM_MS_TO_TICKS(10000), // 10秒超时
                                          pdFALSE,                // 单次触发
                                          (void*)fsm,             // ID存fsm句柄
                                          vIdleTimeoutCallback);
    }
    
    // 初始化上下文
    Main_System_ctx.m1_menu_index = 0;
    Main_System_ctx.run_source = 0;

    fsm_init(fsm, Main_System_evt_buffer, Main_System_QUEUE_SIZE,         
             Main_System_transitions, sizeof(Main_System_transitions)/sizeof(fsm_transition_t),
             STATE_INIT, &Main_System_ctx);
             
    fsm_set_state_callbacks(fsm, Main_System_state_descs, sizeof(Main_System_state_descs)/sizeof(fsm_state_desc_t));

    // 主系统必须订阅所有电机事件，才能响应本地的按键操作和 APP 的网络注入
		// 订阅网络状态就绪事件 (由 fsm_network 模块在收到 OK 后广播)
    event_bus_subscribe(fsm, EVT_NET_STATUS_ONLINE);
		event_bus_subscribe(fsm, EVT_NET_STATUS_ERROR);
    event_bus_subscribe(fsm, EVT_MOTOR_STOP);
    event_bus_subscribe(fsm, EVT_MOTOR_FORWARD);
    event_bus_subscribe(fsm, EVT_MOTOR_BACKWARD);
    event_bus_subscribe(fsm, EVT_MOTOR_LEFT);
    event_bus_subscribe(fsm, EVT_MOTOR_RIGHT);
    event_bus_subscribe(fsm, EVT_MOTOR_ROT_L);
    event_bus_subscribe(fsm, EVT_MOTOR_ROT_R);
		// 订阅安全事件
    event_bus_subscribe(fsm, EVT_WARN_OBSTACLE);
    event_bus_subscribe(fsm, EVT_OBSTACLE_CLEARED);
    on_enter_init(fsm, &Main_System_ctx);
}

/* ================= 主系统任务 ================= */
void Main_System_FSM_task(void *pvParameters)
{
    // 1. 设置主系统 FSM
    Main_System_fsm_setup(&g_Main_System_fsm);
    
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t incoming_evt_id;
    while(1) {
        // --- A. 输入处理：读取按键队列 ---
				if (xKeyLogicQueue != NULL && xQueueReceive(xKeyLogicQueue, &incoming_evt_id, 0) == pdPASS) {
									// 打印调试
									printf("[Main] Recv Direct Evt: %d\r\n", incoming_evt_id);
									// 直接推入 FSM
									fsm_push_event(&g_Main_System_fsm, incoming_evt_id, 0);
				}
        // --- B. 核心逻辑：运行主系统 FSM ---
        fsm_run(&g_Main_System_fsm);    
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

