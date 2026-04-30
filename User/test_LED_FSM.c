#include "test_LED_FSM.h"
#include "fsm_core.h"
#include "FreeRTOS.h"
#include "task.h"
// 定义自动化测试步结构体
typedef struct {
    fsm_event_t event;      // 模拟触发的事件
    uint32_t delay_ticks;   // 触发事件后等待的节拍数 (1 tick = 50ms)
} auto_test_step_t;

// 自动化测试序列：清晰展示三种模式的进入与退出轨迹
static const auto_test_step_t test_sequence[] = {
    // === 场景1：进入模式1 ===
    { EVT_KEY1_SHORT_PRESS,        10 },  // 0. 进菜单，等500ms看UI
    { EVT_KEY2_SHORT_PRESS,  60 },  // 1. 确认选模式1，运行3秒
    { EVT_KEY2_LONG_PRESS,   10 },  // 2. 长按退回菜单，等500ms
    { EVT_KEY2_LONG_PRESS,   20 },  // 3. 长按退回空闲，等1秒

    // === 场景2：进入模式2 ===
    { EVT_KEY1_SHORT_PRESS,        10 },  // 4. 进菜单，等500ms
    { EVT_KEY1_SHORT_PRESS,        10 },  // 5. 切换到选项2，等500ms
    { EVT_KEY2_SHORT_PRESS,  60 },  // 6. 确认选模式2，运行3秒
    { EVT_KEY2_LONG_PRESS,   10 },  // 7. 长按退回菜单，等500ms
    { EVT_KEY2_LONG_PRESS,   20 },  // 8. 长按退回空闲，等1秒

    // === 场景3：进入模式3 ===
    { EVT_KEY1_SHORT_PRESS,        10 },  // 9. 进菜单，等500ms
    { EVT_KEY1_SHORT_PRESS,        10 },  // 10. 切换到选项2，等500ms
    { EVT_KEY1_SHORT_PRESS,        10 },  // 11. 切换到选项3，等500ms
    { EVT_KEY2_SHORT_PRESS,  60 },  // 12. 确认选模式3，运行3秒
    { EVT_KEY2_LONG_PRESS,   10 },  // 13. 长按退回菜单，等500ms
    { EVT_KEY2_LONG_PRESS,   40 }   // 14. 长按退回空闲，等2秒后开始下一轮循环
};

// 状态机全局实例
extern fsm_t g_led_fsm;

void LED_FSM_test_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms 节拍
    
    // 1. 初始化状态机
    fsm_init(&g_led_fsm, STATE_INIT, NULL);
    xLastWakeTime = xTaskGetTickCount();

    // 自动化脚本控制变量
    uint32_t seq_index = 0;
    uint32_t wait_counter = 40; // 初始启动后先等待 2秒 (40 * 50ms) 再开始测试

    const uint32_t seq_length = sizeof(test_sequence) / sizeof(test_sequence[0]);

    while(1)
    {
        /* ==================================================
         * 阶段 A：自动化测试脚本解释器 (Event Producer)
         * ================================================== */
        if (wait_counter > 0) {
            wait_counter--; // 倒计时等待
        } else {
            // 倒计时结束，推送当前序列的事件
            fsm_push_event(&g_led_fsm, test_sequence[seq_index].event);
            // 获取下一步的等待时间
            wait_counter = test_sequence[seq_index].delay_ticks;
            
            // 指向下一个测试步骤，若到达末尾则循环
            seq_index++;
            if (seq_index >= seq_length) {
                seq_index = 0;
            }
        }

        /* ==================================================
         * 阶段 B：状态机引擎路由轮询 (Engine Driver)
         * ================================================== */
        fsm_run(&g_led_fsm);

        /* ==================================================
         * 阶段 C：时域约束控制 (Timing Control)
         * ================================================== */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
