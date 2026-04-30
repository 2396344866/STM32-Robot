#include "dev_ssd1306.h" 
#include "dev_ssd1306_font.h" 
static void SSD1306_SetCursor(SSD1306_Handle_t *h, uint8_t Y, uint8_t X)
{
    h->io.WriteCmd(0xB0 | Y);                   //设置Y位置
    h->io.WriteCmd(0x10 | ((X & 0xF0) >> 4));   //设置X位置高4位
    h->io.WriteCmd(0x00 | (X & 0x0F));          //设置X位置低4位
}

// 内部辅助：次方函数
static uint32_t SSD1306_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void Dev_SSD1306_Init(SSD1306_Handle_t *h)
{
    if (!h || !h->io.WriteCmd) return;

    h->io.InitIO();       // 调用底层的端口初始化
    h->io.DelayMs(200);   // 上电延时

    h->io.WriteCmd(0xAE); //关闭显示
    h->io.WriteCmd(0xD5); //设置显示时钟分频比/振荡器频率
    h->io.WriteCmd(0x80);
    h->io.WriteCmd(0xA8); //设置多路复用率
    h->io.WriteCmd(0x3F);
    h->io.WriteCmd(0xD3); //设置显示偏移
    h->io.WriteCmd(0x00);
    h->io.WriteCmd(0x40); //设置显示开始行
    h->io.WriteCmd(0xA1); //设置左右方向
    h->io.WriteCmd(0xC8); //设置上下方向
    h->io.WriteCmd(0xDA); //设置COM引脚硬件配置
    h->io.WriteCmd(0x12);
    h->io.WriteCmd(0x81); //设置对比度控制
    h->io.WriteCmd(0xCF);
    h->io.WriteCmd(0xD9); //设置预充电周期
    h->io.WriteCmd(0xF1);
    h->io.WriteCmd(0xDB); //设置VCOMH取消选择级别
    h->io.WriteCmd(0x30);
    h->io.WriteCmd(0xA4); //设置整个显示打开/关闭
    h->io.WriteCmd(0xA6); //设置正常/倒转显示
    h->io.WriteCmd(0x8D); //设置充电泵
    h->io.WriteCmd(0x14);
    h->io.WriteCmd(0xAF); //开启显示
    
    Dev_SSD1306_Clear(h);
}

void Dev_SSD1306_Clear(SSD1306_Handle_t *h)
{  
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        SSD1306_SetCursor(h, j, 0);
        for(i = 0; i < 128; i++) {
            h->io.WriteDat(0x00);
        }
    }
}

void Dev_SSD1306_ShowChar(SSD1306_Handle_t *h, uint8_t Line, uint8_t Column, char Char)
{       
    uint8_t i;
    SSD1306_SetCursor(h, (Line - 1) * 2, (Column - 1) * 8);     
    for (i = 0; i < 8; i++) {
        h->io.WriteDat(SSD1306_F8x16[Char - ' '][i]);          
    }
    SSD1306_SetCursor(h, (Line - 1) * 2 + 1, (Column - 1) * 8); 
    for (i = 0; i < 8; i++) {
        h->io.WriteDat(SSD1306_F8x16[Char - ' '][i + 8]);      
    }
}

void Dev_SSD1306_ShowString(SSD1306_Handle_t *h, uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        Dev_SSD1306_ShowChar(h, Line, Column + i, String[i]);
    }
}

void Dev_SSD1306_ShowNum(SSD1306_Handle_t *h, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Dev_SSD1306_ShowChar(h, Line, Column + i, Number / SSD1306_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void Dev_SSD1306_ShowSignedNum(SSD1306_Handle_t *h, uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		Dev_SSD1306_ShowChar(h,Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		Dev_SSD1306_ShowChar(h,Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		Dev_SSD1306_ShowChar(h,Line, Column + i + 1, Number1 / SSD1306_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void Dev_SSD1306_ShowHexNum(SSD1306_Handle_t *h, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / SSD1306_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			Dev_SSD1306_ShowChar(h, Line, Column + i, SingleNumber + '0');
		}
		else
		{
			Dev_SSD1306_ShowChar(h, Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void Dev_SSD1306_ShowBinNum(SSD1306_Handle_t *h, uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		Dev_SSD1306_ShowChar(h, Line, Column + i, Number / SSD1306_Pow(2, Length - i - 1) % 2 + '0');
	}
}

