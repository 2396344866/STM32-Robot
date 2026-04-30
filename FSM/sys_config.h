#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H
#include <stdio.h>
// --- 配置开关 ---
// 如果在 FreeRTOS 环境下编译，请解开此宏或在编译器定义
#define USE_FREERTOS 

#ifdef USE_FREERTOS
    #include "FreeRTOS.h"
    #include "task.h"
    
    #define FSM_GET_TICK()       xTaskGetTickCount()
    #define FSM_MS_TO_TICKS(ms)  pdMS_TO_TICKS(ms)
    #define FSM_DELAY_MS(ms)     vTaskDelay(pdMS_TO_TICKS(ms))
    #define FSM_ENTER_CRITICAL() taskENTER_CRITICAL()
    #define FSM_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
    #define FSM_GET_TICK()       port_get_tick_simulate()
    #define FSM_MS_TO_TICKS(ms)  (ms)
    #define FSM_DELAY_MS(ms)     Sleep(ms) // Windows Sleep (ms)
    #define FSM_ENTER_CRITICAL() // 裸机单线程无需临界区
    #define FSM_EXIT_CRITICAL()  
#endif
/**
 * @def BUS_MAX_SUBS
 * @brief 事件总线最大订阅者数量
 * 
 * 定义了事件总线最多能同时支持的订阅关系数量。
 * 每个订阅关系表示一个状态机对某个事件的关注。
 * !!!是系统能够记录的订阅关系总数，而不是同一时刻活跃的订阅者数量。
 * 
 * @note 配置考虑因素：
 *       - 系统中最活跃的事件数量
 *       - 每个事件的平均订阅者数量
 *       - 总订阅关系数 = (不同事件数) × (每个事件平均订阅者数)
 * 
 * @warning 超过此限制的订阅请求会被静默丢弃！
 * 
 * @par 内存占用计算：
 *      每条订阅记录占用：指针(4/8字节) + uint16_t(2字节) + 对齐填充
 *      总内存占用 ≈ BUS_MAX_SUBS × 8字节（以32位系统为例）
 * 
 * @par 典型配置建议：
 *      - 小型系统（<10个状态机）：16
 *      - 中型系统（10-30个状态机）：32
 *      - 大型系统（>30个状态机）：64或更大
 * 
 * @attention 此值为编译时常量，修改后需要重新编译整个模块
 */
 
#define BUS_MAX_SUBS 48

/**
 * @brief 全局调试开关
 * * 1: 开启调试打印 (printf 有效)
 * 0: 关闭调试打印 (printf 被替换为空语句，节省 Flash 和 串口带宽)
 */
// ================= 调试系统配置 =================

/**
 * @brief 全局调试打印开关
 * 1 = 开启 (输出到串口)
 * 0 = 关闭 (代码被替换为空，不占空间，不输出)
 */
 
#define ENABLE_DEBUG_PRINT 1

#if ENABLE_DEBUG_PRINT
    /**
     * @brief 基础日志宏
     * 自动添加模块标签 tag，但不强制添加换行符(兼容旧代码)
     */
		 
		#define SYS_LOG(tag, fmt, ...)  printf("[%s] " fmt, tag, ##__VA_ARGS__)
    #define LOG_RAW(fmt, ...)       printf(fmt, ##__VA_ARGS__)
    // --- 纯数据打印 (用于电机波形等不带标签的输出) ---
    #define LOG_RAW(fmt, ...)       printf(fmt, ##__VA_ARGS__)
#else
    // --- 关闭状态：宏展开为空语句 ---
    #define SYS_LOG(tag, fmt, ...)  ((void)0)
    #define LOG_RAW(fmt, ...)       ((void)0)
		// 劫持全工程的 printf，彻底屏蔽底层输出
		#define printf(...)             ((void)0)
#endif

// ================= 模块映射宏 =================
// 将这些宏用于你的 .c 文件替换

//// 1. 主控系统日志 (Main System)
//#define LOG_MAIN(fmt, ...)   SYS_LOG("MAIN", fmt, ##__VA_ARGS__)

//// 2. 动作/行为日志 (Action)
//// 替换目标: printf("[Main_System] Action: ...")
//#define LOG_ACT(fmt, ...)    SYS_LOG("ACT ", fmt, ##__VA_ARGS__)

//// 3. 总线/事件日志 (Event Bus)
//#define LOG_BUS(fmt, ...)    SYS_LOG("BUS ", fmt, ##__VA_ARGS__)

//// 4. 电机日志 (Motor)
//#define LOG_MOTOR(fmt, ...)  SYS_LOG("MOTO", fmt, ##__VA_ARGS__)

#endif
