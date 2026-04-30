#include "hal_delay.h"

// 引入 FreeRTOS 头文件以获取调度器状态
#ifdef HAL_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* ==========================================
 * 私有变量
 * ========================================== */
static uint32_t fac_us = 0;      // SysTick 模式下的倍频因子
static uint32_t fac_us_soft = 0; // 软件循环模式下的倍频因子

/* ==========================================
 * 接口实现
 * ========================================== */

void HAL_Delay_Init(void)
{
    // 更新系统时钟，确保 SystemCoreClock 正确
    SystemCoreClockUpdate();

    // 1. 计算 SysTick 模式下的倍频因子 (用于调度器启动后)
    // SysTick 频率通常等于 HCLK (72MHz) 或 HCLK/8
    // FreeRTOS 默认配置通常是 HCLK。
    fac_us = SystemCoreClock / 1000000;

    // 2. 计算软件循环模式下的倍频因子 (用于调度器启动前)
    // 经验值：在 72MHz 下，简单的 while(--) 循环大概需要 4~5 个时钟周期
    // 为了保证延时只多不少（OLED 初始化宁可慢不可快），我们除以 5 作为估算
    fac_us_soft = (uint32_t)(SystemCoreClock / 1000000) / 5;
    
    // 防止主频过低导致为0
    if(fac_us_soft < 1) fac_us_soft = 1;
}

void HAL_Delay_us(uint32_t us)
{
    // 临时变量定义
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload;
    uint32_t i;

    // -----------------------------------------------------------
    // 状态判断：检查 FreeRTOS 调度器是否已经开始运行
    // -----------------------------------------------------------
    uint8_t scheduler_running = 0;

#ifdef HAL_USE_FREERTOS
    // 如果定义了 FreeRTOS，检查调度器状态
    // xTaskGetSchedulerState 可以在调度器启动前安全调用
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        scheduler_running = 1;
    }
#endif

    // -----------------------------------------------------------
    // 分支 A: 调度器未启动 (SysTick 未工作) -> 使用软件空循环
    // -----------------------------------------------------------
    if (scheduler_running == 0)
    {
        // 这是一个粗略的延时，仅用于初始化阶段
        while(us--)
        {
            // 软件空循环
            for(i = 0; i < fac_us_soft; i++) {
                __NOP(); // 避免编译器过度优化空循环
            }
        }
    }
    // -----------------------------------------------------------
    // 分支 B: 调度器已启动 (SysTick 已工作) -> 使用寄存器差值计算
    // -----------------------------------------------------------
    else
    {
        reload = SysTick->LOAD; 
        ticks = us * fac_us; 
        told = SysTick->VAL; 
        
        while (1)
        {
            tnow = SysTick->VAL; 
            if (tnow != told)
            {
                if (tnow < told)
                    tcnt += told - tnow; 
                else
                    tcnt += reload - tnow + told; 
                
                told = tnow;
                if (tcnt >= ticks) break; 
            }
        }
    }
}

void HAL_Delay_ms(uint32_t ms)
{
#ifdef HAL_USE_FREERTOS
    // 如果调度器运行中 且 不在中断里 -> 使用系统阻塞延时 (释放 CPU)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED && xPortIsInsideInterrupt() == pdFALSE)
    {
        vTaskDelay(pdMS_TO_TICKS(ms)); 
    }
    else
#endif
    {
        // 调度器未启动 或 在中断中 -> 使用微秒级死循环
        while(ms--)
        {
            HAL_Delay_us(1000);
        }
    }
}

uint32_t HAL_GetTick(void)
{
#ifdef HAL_USE_FREERTOS
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        return (uint32_t)xTaskGetTickCount();
    }
#endif
    return 0;
}
