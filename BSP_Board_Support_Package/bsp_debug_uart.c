#include "bsp_debug_uart.h"
#include "sys_config.h"


#if ENABLE_DEBUG_PRINT
	#include "hal_uart.h"
	#include "hal_uart_dma.h"
	#include "hal_gpio.h"

	#define DEBUG_RX_MAX 128
	uint8_t g_debug_rx_buf[DEBUG_RX_MAX];
	QueueHandle_t xDebugRxQueue = NULL; // 定义调试队列
	
	// 注意：此处去掉了 static，以便下方的 fputc 可以访问到
	UART_Handle_t hDebugUart = {
			.Instance = USART2,
			.BaudRate = 115200 
	};
#endif


/* ================== 2. 必须全局保留的半主机模式禁用代码 ================== */
// 只要工程中使用了 snprintf / sprintf，就必须禁用半主机模式，否则裸机运行直接死机！
#pragma import(__use_no_semihosting)

struct __FILE {
    int handle;
};
FILE __stdout;

void _sys_exit(int x) {
    x = x;
}

// 【核心修复】：fputc 必须放在宏的外面全局可见！用来满足链接器的需求。
int fputc(int ch, FILE *f)
{
#if ENABLE_DEBUG_PRINT
    // 如果开启调试，则真正通过串口发送
    HAL_UART_SendByte(&hDebugUart, (uint8_t)ch);
#endif
    // 如果关闭调试，这就是个不执行任何硬件操作的空函数
    return ch;
}
/* ==================================================================== */


#if ENABLE_DEBUG_PRINT
	void BSP_DebugUART_Init(uint32_t baudrate)
	{
			hDebugUart.BaudRate = baudrate;
		// 创建调试解析队列
			if (xDebugRxQueue == NULL) {
					xDebugRxQueue = xQueueCreate(5, sizeof(uint16_t));
			}
		  // 初始化 PA2(TX), PA3(RX)
			HAL_GPIO_Init(GPIOA, GPIO_Pin_2, HAL_GPIO_MODE_AF_PP);
			HAL_GPIO_Init(GPIOA, GPIO_Pin_3, HAL_GPIO_MODE_INPUT_PU);

			HAL_UART_Init(&hDebugUart);
			HAL_UART_DMA_Rx_Init(&hDebugUart, g_debug_rx_buf, DEBUG_RX_MAX);
			HAL_UART_EnableIRQ(&hDebugUart, 6);
	}
	// 独占 USART2 中断
	void USART2_IRQHandler(void)
	{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			if (USART_GetFlagStatus(USART2, USART_FLAG_IDLE) != RESET)
			{
					volatile uint32_t temp = USART2->SR;
					temp = USART2->DR;
					(void)temp;

					DMA_Cmd(DMA1_Channel6, DISABLE);
					uint16_t rx_len = DEBUG_RX_MAX - DMA_GetCurrDataCounter(DMA1_Channel6);

					if (rx_len > 0) {
							// 确保字符串安全截断
							if (rx_len < DEBUG_RX_MAX) g_debug_rx_buf[rx_len] = '\0';
							else g_debug_rx_buf[DEBUG_RX_MAX - 1] = '\0';
							
							// 将数据长度发送至任务队列
							if (xDebugRxQueue != NULL) {
									xQueueSendFromISR(xDebugRxQueue, &rx_len, &xHigherPriorityTaskWoken);
							}
					}

					DMA_SetCurrDataCounter(DMA1_Channel6, DEBUG_RX_MAX);
					DMA_Cmd(DMA1_Channel6, ENABLE);
			}
			
			if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET || 
					USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) 
			{
					volatile uint32_t temp = USART2->SR;
					temp = USART2->DR; 
					(void)temp;
			}
			
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
#endif
