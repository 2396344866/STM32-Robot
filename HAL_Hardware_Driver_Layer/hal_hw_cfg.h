#ifndef __HAL_HW_CFG_H
#define __HAL_HW_CFG_H

/*
 * @file hal_hw_cfg.h
 * @brief 集中封装工程中的"魔法数字"（Magic Numbers）
 *
 * 问题背景：原代码散落大量无含义的字面量（延时计数、超时、阈值、脉冲数），
 * 例如 I2C 复位的 9 个时钟脉冲、HAL_Delay_us(5)、ADC 采样周期、看门狗栈水位 64、
 * 传感器同步超时 50ms、避障距离阈值 2.0/15.0/17.0 等。本文件统一提取为带注释的
 * 宏 / 枚举，便于维护与评审，避免"数字含义靠猜"。
 *
 * 命名约定：HAL_HW_CFG_*（硬件时序 / 阈值类）
 * 作用域：被 HAL / BSP / FSM / User 各层包含，全部为编译期常量，零运行时开销。
 */

#include "stm32f10x.h"

/* ===================== I2C 总线恢复（HAL_HardI2C_ResetBus） ===================== */
/* 总线死锁时，主机连续输出 9 个 SCL 脉冲使从机位计数器溢出释放 SDA（I2C 规范：
 * 每字节 8 bit + 1 ACK，9 拍可强制从机放弃总线）。 */
#define HAL_HW_CFG_I2C_RESET_CLK_PULSES   9U
/* 单拍 SCL 高/低电平保持时间（us）。MPU6050 最快 400kHz => 周期 2.5us，
 * 取 5us 留足余量，纯 GPIO 翻转无需精确，但不可为 0 否则脉冲无效。 */
#define HAL_HW_CFG_I2C_RESET_PULSE_US     5U

/* ===================== ADC / DMA ===================== */
/* ADC 规则组采样时间（cycles）。239.5 cycles 为 F103 最慢档，
 * 对高阻抗传感器（MQ2 分压、光敏）可降低采样电容充电误差。 */
#define HAL_HW_CFG_ADC_SAMPLE_CYCLES      ADC_SampleTime_239Cycles5
/* ADC 双缓冲单帧容量上限（ChannelCount<=2 时即 4 字）。定义为编译期上限，
 * 防止 DMABuffer 静态数组越界（bsp_sensor.c 的 adc_dma_buffer[4]）。 */
#define HAL_HW_CFG_ADC_DMA_BUF_WORDS      4U

/* ===================== 传感器同步 / 超时 ===================== */
/* DMA 半满/全满事件等待超时（ms）。超时则保留上一帧（降级而非阻塞），
 * 对应 bsp_sensor.c 的 pdMS_TO_TICKS(50)。 */
#define HAL_HW_CFG_SENSOR_SYNC_TIMEOUT_MS 50U
/* 传感器数据提交互斥量获取超时（ms），对应 fsm_sensor.c 的 pdMS_TO_TICKS(10)。 */
#define HAL_HW_CFG_SENSOR_MUTEX_TIMEOUT_MS 10U
/* DHT11 降频读取分频：每 20 次轮询读一次（温湿度变化慢，无需每周期读）。 */
#define HAL_HW_CFG_DHT11_POLL_DIV         20U

/* ===================== 超声波避障阈值（cm） ===================== */
/* 进入避障警告的近距区间 (2, 15)cm；>= 17cm 判定障碍已清除。
 * 阈值来自机器人物理尺寸与超声波盲区（<2cm 测不准，15~17cm 为滞回带防抖）。 */
#define HAL_HW_CFG_AVOID_NEAR_MIN_CM      2.0f
#define HAL_HW_CFG_AVOID_NEAR_MAX_CM      15.0f
#define HAL_HW_CFG_AVOID_CLEAR_CM         17.0f

/* ===================== 看门狗 / 健康监控 ===================== */
/* 栈高水位阈值（FreeRTOS 字 = 4 字节）。低于该值视为"警告"，
 * 对应 main.c watchdog_task 的 wm < 64。 */
#define HAL_HW_CFG_WDG_STACK_WATERMARK    64U
/* 看门狗监护周期（ms）。1s 周期不长期阻塞空闲任务，保证 tickless 能进入休眠。 */
#define HAL_HW_CFG_WDG_PERIOD_MS          1000U
/* 连续异常次数达到该值进入"故障"级（停电机+闪 LED，可扩展 NVIC_SystemReset）。 */
#define HAL_HW_CFG_WDG_FAULT_STREAK       3U

/* ===================== 系统启动 ===================== */
/* SysTick 节拍：1ms（SystemCoreClock/1000）。用于 HAL_Delay / FreeRTOS 时基。 */
#define HAL_HW_CFG_TICK_MS                1U


/* ===================== 参数校验（assert_param） ===================== */
/* 复用 FreeRTOS 的 configASSERT 作为底层机制：
 *   - 在 FreeRTOSConfig.h 中定义 configASSERT(x) 时（调试构建），断言失败进入
 *     while(1) / 断点，立即暴露非法入参（指针空、长度 0、句柄无效等）；
 *   - 若未定义 configASSERT（生产构建），则本宏展开为空，零开销。
 * 用法：API 入口处 assert_param(handle != NULL);  assert_param(Size > 0);
 * 注意：assert_param 仅用于"绝不该发生"的不变量校验，不可用于可预期的运行时错误
 *       （如总线超时、DMA 错误），后者应走正常的错误码返回 / 安全模式分支。
 */
#ifndef assert_param
    #ifdef configASSERT
        #define assert_param(expr)    configASSERT(expr)
    #else
        #define assert_param(expr)    ((void)0)
    #endif
#endif

#endif /* __HAL_HW_CFG_H */
