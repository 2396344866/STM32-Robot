#include "fsm_key.h"
#include "sys_events.h"
#include "timers.h"

/* 外部队列引用 */
extern QueueHandle_t xKeyLogicQueue;

/* 定时器句柄 */
static TimerHandle_t xKeyScanTimer = NULL;

/* 按键结构体 - 用于管理每个按键的状态 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    uint16_t      press_cnt;      // 按下计时器 (1 tick = 10ms)
    uint8_t       state;          // 0:松开, 1:按下
    sys_event_t   short_event;    // 短按发送的事件
    sys_event_t   long_event;     // 长按发送的事件
} key_obj_t;

/* 定义按键对象 */
static key_obj_t keys[2] = {
		//GPIO 引脚 计时 松开按下 短按发送 长按发送
    {KEY1_GPIO_PORT, KEY1_GPIO_PIN, 0, 0, EVT_KEY1_SHORT_PRESS, EVT_KEY1_LONG_PRESS},
    {KEY2_GPIO_PORT, KEY2_GPIO_PIN, 0, 0, EVT_KEY2_SHORT_PRESS, EVT_KEY2_LONG_PRESS}
};

/* ================= 核心扫描逻辑 (每10ms运行一次) ================= */
static void KEY_Scan_StateMachine(key_obj_t *k)
{
    // 1. 读取物理引脚 (低电平有效)
    uint8_t is_pressed = (GPIO_ReadInputDataBit(k->port, k->pin) == Bit_RESET);

    if (is_pressed) {
        // --- 按下状态 ---
        if (k->press_cnt < 0xFFFF) {
            k->press_cnt++; 
        }

        // 检查长按 (One-shot: 仅在计数器刚好等于阈值时触发)
        if (k->press_cnt == KEY_LONG_TICKS) {
            sys_event_t evt = k->long_event;
            xQueueSend(xKeyLogicQueue, &evt, 0);
        }
    } 
    else {
        // --- 松开状态 (或抖动高电平) ---
        
        // 只有当计数器超过消抖阈值，且未达到长按阈值时，才判定为短按
        if (k->press_cnt >= KEY_DEBOUNCE_TICKS && k->press_cnt < KEY_LONG_TICKS) {
            sys_event_t evt = k->short_event;
            xQueueSend(xKeyLogicQueue, &evt, 0);
        }

        // 复位计数器
        k->press_cnt = 0;
    }
}

/* ================= 定时器回调 ================= */
static void vKeyScanCallback(TimerHandle_t xTimer) {
    // 轮询所有按键
    for (int i = 0; i < 2; i++) {
        KEY_Scan_StateMachine(&keys[i]);
    }
}




/* ================= 初始化 ================= */
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 时钟使能 (只开启 GPIO 时钟，不再需要 AFIO)
    RCC_APB2PeriphClockCmd(KEY1_GPIO_CLK | KEY2_GPIO_CLK, ENABLE);

    // 2. GPIO 配置 (上拉输入)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = KEY1_GPIO_PIN;
    GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = KEY2_GPIO_PIN;
    GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);

    // 3. 创建并启动扫描定时器
    // 周期: 10ms, 自动重装载: pdTRUE
    xKeyScanTimer = xTimerCreate(
			"KeyScan",                          // 1. 定时器名字 (调试用)
			pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS),  // 2. 定时周期 (转换为Tick数)
			pdTRUE,                             // 3. 自动重装载 (pdTRUE=周期性, pdFALSE=单次)
			NULL,                               // 4. 定时器ID (用于同回调区分不同定时器，此处不用)
			vKeyScanCallback                    // 5. 回调函数 (超时后执行的函数)
		);

    if (xKeyScanTimer != NULL) {
				  // 创建成功，启动定时器 (等待时间设为0)
        xTimerStart(xKeyScanTimer, 0);
    }
}
