#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
/* --- HAL层 ---*/
#include "HAL_delay.h"
/* --- BSP层 ---*/
#include "bsp_LED.h"
#include "bsp_debug_uart.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_mpu6050.h"
#include "bsp_esp8266.h"
/* --- 业务解耦模块 --- */
#include "sys_events.h"
#include "fsm_core.h"
#include "fsm_key.h"         // 包含用户按键输入（切换状态）
#include "fsm_motor.h"       // 包含 Motor_FSM_Setup
#include "fsm_main_system.h" // 包含 Main_System_fsm_setup
#include "fsm_network.h"			// 包含 Network_FSM_Setup
#include "fsm_sensor.h"
#include "event_bus.h"       // 用于总线初始化
#include "state_repo.h"      // 中央状态仓库（层2 状态数据层）
#include "sys_config.h"
#include "hal_hw_cfg.h"  // 魔法数字宏 + assert_param
#include "stm32f10x_iwdg.h"  // 硬件独立看门狗
#include "stm32f10x_pwr.h"   // PVD 电源电压检测器
#include "stm32f10x_exti.h"  // EXTI_Line16 连接 PVD 输出
#include "bsp_servo.h"
#include "fsm_motor.h"       // enter_safe_state / g_fault_safe_active（电源欠压安全态共用）
/* ================= 任务配置 ================= */
#define START_TASK_PRIO         1
#define START_STK_SIZE          512
#define MOTOR_TASK_PRIO         5
#define MOTOR_STK_SIZE          256
#define MAIN_SYS_TASK_PRIO      3
#define MAIN_SYS_STK_SIZE       256  
#define NETWORK_TASK_PRIO       4  
#define NETWORK_STK_SIZE        512 
#define SENSOR_TASK_PRIO   2 
#define SENSOR_STK_SIZE    256
TaskHandle_t StartTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t MainSysTask_Handler;
TaskHandle_t NetworkTask_Handler;
TaskHandle_t SensorTask_Handler;
// 全局按键逻辑队列 (跨模块通信用，属于 OS 级资源)
QueueHandle_t xKeyLogicQueue;

/* 看门狗/健康监控任务：周期性检查各业务任务是否仍Alive，
 * 并统计最小栈剩余水位，发现异常进入安全模式或触发复位。 */
#define WDG_TASK_PRIO   1
#define WDG_STK_SIZE    256
TaskHandle_t WatchdogTask_Handler;

/* 栈溢出钩子：溢出即停机并打印，避免错误静默扩散 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask; (void)pcTaskName;
    /* 注意：本钩子在 FreeRTOS 上下文切换路径调用，此时任务栈已溢出，
     * 严禁再调用 printf（会二次压栈直接崩溃）。仅做不可恢复停机标记，
     * 由 IWDG 最后防线超时硬复位，或调试器在此设断点。 */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/* 断言失败陷阱：configASSERT / assert_param 在此落地。
 * 调试构建下非法入参或内核不变量违例立即暴露位置并停机，绝不静默跑飞。
 * 安全约束：不得调用 printf —— configASSERT 可能被 ISR 上下文触发
 *（如 port.c 临界区/退出临界区的断言），printf 依赖半主机/串口且不可重入，
 * 在中断内调用会导致 HardFault 或死锁。 */
void assert_failed(uint8_t* file, uint32_t line) {
    (void)file; (void)line;
    taskDISABLE_INTERRUPTS();
    while (1);  // 停机陷阱，交给定时硬复位或调试器断点介入
}

/* 堆分配失败钩子：heap_4 动态分配（队列/事件组/任务通知等）在
 * configTOTAL_HEAP_SIZE(10KB) 耗尽时回调。动态分配失败属不可恢复错误，
 * 必须 fail-fast 停机而非返回 NULL 让调用方静默空指针解引用。 */
void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    while (1);
}

/* 硬件独立看门狗 IWDG：作为任务级 watchdog_task 之上的最后防线。
 * 超时(约2s)长于任务级监护周期(1s)；仅当 watchdog_task 周期性确认各任务健康后才喂狗，
 * 若任务级看门狗本身被死锁卡死，IWDG 仍能在超时后硬复位 MCU。
 * 采用标准外设库 IWDG 驱动（非 HAL）。LSI 40kHz：/64 分频=625Hz，重装 1250 => 2s 超时。 */
static void IWDG_Init_Hardware(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  // 解锁 PR/RLR 写权限
    IWDG_SetPrescaler(IWDG_Prescaler_64);          // 40kHz / 64 = 625Hz
    IWDG_SetReload(1250);                           // 1250 / 625 = 2s 超时
    IWDG_ReloadCounter();                           // 先喂一次，避免立即复位
    IWDG_Enable();                                  // 启动独立看门狗（一旦开启不可关闭，除非复位）
}

/* P0-② 电源 UVLO：片内 PVD（可编程电压检测器）监测 VDD(3.3V)。
 * 阈值 2.9V：当系统电压跌穿（如 5V 电池严重不足经 LDO 拉垮 3.3V）时 PVDO 标志置位。
 * 注意边界：PVD 监测 VDD 而非电池 5V；对"5V 不足但 3.3V 暂稳"的中间态（M1 原场景）
 * 不直接触发，需硬件分压采样电池 5V 才能根治——此处作为系统级最后电压防线。
 * 检测到欠压 → 进安全态收腿趴下 + 不再喂 IWDG → 由硬件看门狗 2s 硬复位，
 * 避免驱动在低压下抽搐触发不可控复位（M1 根因治理）。 */
static void PVD_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  // PVD 位于 PWR 外设
    PWR_PVDLevelConfig(PWR_PVDLevel_2V9);                 // 阈值 2.9V（LDO 跌穿前预警）
    PWR_PVDCmd(ENABLE);
    // 配置 EXTI_Line16（PVD 输出）为中断或事件；此处用轮询 PVDO 标志，不使能 EXTI 中断
    EXTI_ClearITPendingBit(EXTI_Line16);
}

/* 查询式欠压检测：返回 1 表示 VDD 已跌穿 PVD 阈值 */
static inline uint8_t pvd_undervoltage(void) {
    return (PWR_GetFlagStatus(PWR_FLAG_PVDO) == SET) ? 1 : 0;
}

/* 静态分配回调：configSUPPORT_STATIC_ALLOCATION=1 时 FreeRTOS 要求提供，
 * 空闲任务与定时器任务改用静态数组，运行期零动态分配（安全关键场景）。 */
static StackType_t s_idle_stk[configMINIMAL_STACK_SIZE];
static StaticTask_t s_idle_tcb;
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer   = &s_idle_tcb;
    *ppxIdleTaskStackBuffer = s_idle_stk;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}
static StackType_t s_timer_stk[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t s_timer_tcb;
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer   = &s_timer_tcb;
    *ppxTimerTaskStackBuffer = s_timer_stk;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

void watchdog_task(void *pvParameters);
void start_task(void *pvParameters);
//// 全局 FSM 句柄
//fsm_t g_Main_System_fsm;
//// 按键逻辑队列
//QueueHandle_t xKeyLogicQueue;
//// 全局存放姿态数据的结构体
//MPU6050_Data_t g_imu_data;
//// 任务函数声明
//void start_task(void *pvParameters);
//void Motor_FSM_task(void *pvParameters);
//void Main_System_FSM_task(void *pvParameters);
//void Network_FSM_Task(void *pvParameters);

int main()
{
		// 硬件层初始化 (必须在调度器启动前完成)
    // 确保你的中断分组设置为 Group 4 (FreeRTOS 推荐)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		HAL_Delay_Init();
		BSP_LED_Init();
		IWDG_Init_Hardware();   // 硬件独立看门狗上电即开，作为最后防线
		PVD_Init();             // 片内电压检测器（UVLO 系统级防线）

		// 参数/环境校验：时钟树与关键外设就绪前不得进入调度
		if (SysTick_Config(SystemCoreClock / 1000) != 0) {
				while(1);  // SysTick 配置失败，静默死锁风险拦截
		}

		//创建开始任务
    if (xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&StartTask_Handler) != pdPASS) {
				while(1);  // 开始任务都建不出，系统无意义
		}

		vTaskStartScheduler();          //开启任务调度，创建一个空闲任务（IDLE Task）
		// 仅当调度器启动失败（如堆耗尽）才会返回到这里
		while(1){};
}

void start_task(void *pvParameters){
    // 1. 进入临界区，保护创建过程
		taskENTER_CRITICAL();           
    // --- A. 资源分配 (优先级最高) ---
    // 创建 Key 事件队列
    xKeyLogicQueue = xQueueCreate(10, sizeof(sys_event_t));
    
    // 严格的错误检查：如果核心队列创建失败，系统不能启动
    if(xKeyLogicQueue == NULL) {
        printf("Queue Creation Failed! System Halted.\r\n");
        // 陷入死循环或触发硬件复位，防止后续空指针操作
        while(1) {}; 
    }
    // 初始化总线，清空订阅表
    event_bus_init();
    // 初始化中央状态仓库（事件组 + 全局状态表），必须在各 FSM 写入前创建
    state_repo_init();
		// --- B. 中断硬件初始化(硬件上电死机大概率就是提前开启中断 中断触发导致)---
    // 必须在 xKeyLogicQueue 创建成功  后调用，否则中断中写队列会 HardFault
		KEY_Init();
		#if ENABLE_DEBUG_PRINT
				BSP_DebugUART_Init(115200, NetworkTask_Handler); /* 调试模式：只初始化串口 */
				printf("BSP_DebugUART_Init Complete\n");
		#endif
		BSP_Servo_Init();           /* 运行模式：只初始化舵机 */
		printf("BSP_Servo_Init Complete\n");
		BSP_ESP8266_Init(115200, NetworkTask_Handler);  // 注册唤醒目标，ISR 用 xTaskNotify 替代二值信号量
    printf("BSP_ESP8266_Init Complete\n");
		BSP_OLED_Init();
		BSP_OLED_ShowString(1,1,"OK");
		printf("BSP_OLED_Init\n");
		BSP_MPU6050_Init(); 
		printf("BSP_MPU6050_Init\n");
    // --- C. 创建业务任务 ---
    // 电机任务
    if (xTaskCreate((TaskFunction_t )Motor_FSM_task,
                (const char* )"Motor_Task",
                (uint16_t       )MOTOR_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )MOTOR_TASK_PRIO,
                (TaskHandle_t* )&MotorTask_Handler) != pdPASS) {
        printf("[FATAL] xTaskCreate Motor_Task failed! System Halted.\r\n");
        while(1);
    }
    // 主系统任务
    if (xTaskCreate((TaskFunction_t )Main_System_FSM_task,
                (const char* )"Main_Task",
                (uint16_t       )MAIN_SYS_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )MAIN_SYS_TASK_PRIO,
                (TaskHandle_t* )&MainSysTask_Handler) != pdPASS) {
        printf("[FATAL] xTaskCreate Main_Task failed! System Halted.\r\n");
        while(1);
    }
		// 网络处理任务
    if (xTaskCreate((TaskFunction_t )Network_FSM_Task,
                (const char* )"Net_Task",
                (uint16_t       )NETWORK_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )NETWORK_TASK_PRIO,
                (TaskHandle_t* )&NetworkTask_Handler) != pdPASS) {
        printf("[FATAL] xTaskCreate Net_Task failed! System Halted.\r\n");
        while(1);
    }
		// 传感器轮询 ---
    if (xTaskCreate((TaskFunction_t )Sensor_FSM_Task,
                (const char* )"Sensor_Task",
                (uint16_t       )SENSOR_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )SENSOR_TASK_PRIO,
                (TaskHandle_t* )&SensorTask_Handler) != pdPASS) {
        printf("[FATAL] xTaskCreate Sensor_Task failed! System Halted.\r\n");
        while(1);
    }
		// 看门狗/健康监控任务
    if (xTaskCreate((TaskFunction_t )watchdog_task,
                (const char* )"Watchdog",
                (uint16_t       )WDG_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )WDG_TASK_PRIO,
                (TaskHandle_t* )&WatchdogTask_Handler) != pdPASS) {
        printf("[FATAL] xTaskCreate Watchdog failed! System Halted.\r\n");
        while(1);
    }						
    // --- D. 退出与清理 ---
    // 退出临界区-恢复中断
    taskEXIT_CRITICAL();
    // 最后删除创建任务函数
    vTaskDelete(StartTask_Handler);
}

/*
 * 看门狗 / 健康监控任务
 * 四级运行健康监护：
 *   正常  —— 各任务 eTaskGetState == Running/Ready/Suspended（非Deleted），栈水位充足
 *   警告  —— 某任务栈水位低于阈值(如<64字)，打印告警，不干预
 *   刹车  —— 某业务任务意外进入 Deleted/Invalid 状态（被误删/崩溃），尝试重建或挂起整机
 *   故障  —— 连续多次(见 HAL_HW_CFG_WDG_FAULT_STREAK)检测失败，进入安全模式（停电机、闪LED、可触发 NVIC_SystemReset）
 * 依赖 INCLUDE_eTaskGetState（已在 FreeRTOSConfig.h 开启）。
 */
static const struct {
    const char* name;
    TaskHandle_t* ph;
} s_wdg_targets[] = {
    {"Motor_Task",   &MotorTask_Handler},
    {"Main_Task",    &MainSysTask_Handler},
    {"Net_Task",     &NetworkTask_Handler},
    {"Sensor_Task",  &SensorTask_Handler},
};

void watchdog_task(void *pvParameters) {
    (void)pvParameters;
    uint32_t fail_streak = 0;
    uint32_t last_drop = 0;        // 事件总线累计丢失计数（仅观测，不触发复位）
    uint32_t last_sub_full = 0;    // 订阅表满计数

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(HAL_HW_CFG_WDG_PERIOD_MS));   // 1s 监护周期，不长期阻塞空闲任务(保证 tickless 休眠)

        /* 事件总线可观测性：安全关键事件（如 EVT_WARN_OBSTACLE）若因订阅者
         * 队列满被静默丢弃，此处能发现并打印，便于现场定位；队列满多为瞬时
         * 拥塞，不直接复位，避免误杀。sub_full 反映订阅表容量不足（需扩容）。 */
        event_bus_stats_t eb;
        event_bus_get_stats(&eb);
        if (eb.drop_total != last_drop) {
            printf("[WDG][事件总线] 事件投递丢失 +%lu (累计 %lu)\n",
                   (unsigned long)(eb.drop_total - last_drop), (unsigned long)eb.drop_total);
            last_drop = eb.drop_total;
        }
        if (eb.sub_full_total != last_sub_full) {
            printf("[WDG][事件总线] 订阅表已满，订阅失败 +%lu\n",
                   (unsigned long)(eb.sub_full_total - last_sub_full));
            last_sub_full = eb.sub_full_total;
        }

        /* P0-② UVLO：片内 PVD 欠压检测。跌穿 → 进安全态收腿趴下，并停止喂 IWDG，
         * 由硬件看门狗 2s 硬复位，避免低压驱动抽搐触发不可控复位（M1 根因治理）。 */
        if (pvd_undervoltage()) {
            if (!g_fault_safe_active) {
                enter_safe_state();
                printf("[PVD][欠压] VDD 跌穿阈值，进入安全态\n");
            }
            continue;   // 欠压期不喂狗 → 触发硬复位，而非带病运行
        }

        uint8_t fault = 0;
        for (size_t i = 0; i < sizeof(s_wdg_targets)/sizeof(s_wdg_targets[0]); i++) {
            TaskHandle_t h = *s_wdg_targets[i].ph;
            if (h == NULL) { fault = 1; continue; }
            eTaskState st = eTaskGetState(h);
            if (st == eDeleted || st == eInvalid) {
                printf("[WDG][BRAKE] task %s state abnormal (%d)\n", s_wdg_targets[i].name, st);
                fault = 1;
            }
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
            UBaseType_t wm = uxTaskGetStackHighWaterMark(h);
            if (wm < HAL_HW_CFG_WDG_STACK_WATERMARK) {
                printf("[WDG][警告] 任务 %s 栈剩余水位低: %u 字\n", s_wdg_targets[i].name, wm);
            }
#endif
        }

        if (fault) {
            if (++fail_streak >= 3) {
                printf("[WDG][故障] 连续3次异常，进入安全模式\n");
                // 安全模式：闪 LED 提示故障（硬件复位可在此调用 NVIC_SystemReset()）
                BSP_LED1_On();
                vTaskDelay(pdMS_TO_TICKS(50));
                BSP_LED1_Off();
                // 注意：此处不喂 IWDG —— 持续故障应由硬件看门狗超时硬复位
            }
        } else {
            fail_streak = 0;
            IWDG_ReloadCounter();  // 各任务健康，喂硬件看门狗（最后防线）
        }
    }
}
