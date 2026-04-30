#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#include <stdint.h>

// 初始化所有的舵机硬件(GPIO, Timer)和句柄
void BSP_Servo_Init(void);

// 具体的关节控制接口 (应用层调用这些)
void BSP_Servo_Set_Left_Top_Knee(float angle);
void BSP_Servo_Set_Left_Top_Hip(float angle);

void BSP_Servo_Set_Right_Top_Knee(float angle);
void BSP_Servo_Set_Right_Top_Hip(float angle);

void BSP_Servo_Set_Right_Bottom_Knee(float angle);
void BSP_Servo_Set_Right_Bottom_Hip(float angle);

void BSP_Servo_Set_Left_Bottom_Knee(float angle);
void BSP_Servo_Set_Left_Bottom_Hip(float angle);

#endif
