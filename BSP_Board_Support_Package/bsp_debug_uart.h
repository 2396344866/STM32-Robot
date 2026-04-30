#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H

#include "sys_config.h"

#if ENABLE_DEBUG_PRINT
    #include <stdint.h>
    #include "FreeRTOS.h"
    #include "queue.h"

    extern QueueHandle_t xDebugRxQueue; 
    extern uint8_t g_debug_rx_buf[];
    void BSP_DebugUART_Init(uint32_t baudrate);
    void BSP_DebugUART_SendString(char* str);
#else
    // 调试关闭时，将对外接口宏替换为空，实现无缝裁剪
    #define BSP_DebugUART_Init(baudrate)   ((void)0)
    #define BSP_DebugUART_SendString(str)  ((void)0)
#endif

#endif
