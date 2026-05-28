#ifndef __BSP_ESP8266_H
#define __BSP_ESP8266_H

#include "FreeRTOS.h"
#include "queue.h"

#define ESP8266_RX_MAX 256

extern uint8_t g_esp8266_rx_buf[ESP8266_RX_MAX];
extern QueueHandle_t xNetRxQueue;

void BSP_ESP8266_Init(uint32_t baudrate);
void BSP_ESP8266_SendString(const char* str);

#endif
