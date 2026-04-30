#ifndef __DEV_SSD1306_H
#define __DEV_SSD1306_H
#include <stdint.h>

// 1. 定义硬件抽象接口 (V-Table)
typedef struct {
    void (*InitIO)(void);           // 初始化IO
    void (*WriteDat)(uint8_t data); // 写数据
    void (*WriteCmd)(uint8_t cmd);  // 写命令
    void (*DelayMs)(uint32_t ms);   // 延时
} SSD1306_IO_Interface_t;

// 2. 定义设备句柄
typedef struct {
    SSD1306_IO_Interface_t io;
} SSD1306_Handle_t;

// 3. 逻辑功能声明
void Dev_SSD1306_Init(SSD1306_Handle_t *handle);
void Dev_SSD1306_Clear(SSD1306_Handle_t *handle);
void Dev_SSD1306_ShowChar(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, char Char);
void Dev_SSD1306_ShowString(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, char *String);
void Dev_SSD1306_ShowNum(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void Dev_SSD1306_ShowSignedNum(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void Dev_SSD1306_ShowNum(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void Dev_SSD1306_ShowHexNum(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void Dev_SSD1306_ShowBinNum(SSD1306_Handle_t *handle, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
#endif
