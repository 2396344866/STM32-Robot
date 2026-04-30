#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include <stdint.h>

/* * 初始化 OLED 及其底层 I2C 总线 
 * 注意：使用前需确保 SystemCoreClock 更新及 Delay 函数可用
 */
void BSP_OLED_Init(void);

/* ==========================================
 * 应用层绘图接口 (代理模式)
 * ========================================== */

void BSP_OLED_Clear(void);

void BSP_OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

void BSP_OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

void BSP_OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

void BSP_OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

void BSP_OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

void BSP_OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif /* __BSP_OLED_H */
