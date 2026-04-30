#include "bsp_servo.h"
#include "dev_servo.h"
#include "hal_pwm.h"
#include "hal_gpio.h"

// ==========================================
// 1. 硬件适配层 (Glue Code)
// 将 HAL 的 TIM 操作封装成 dev_servo 需要的函数指针格式
// ==========================================

// TIM2 Channels
static void TIM2_CH1_Write(uint16_t val) { HAL_PWM_SetCompare(TIM2, 1, val); }
static void TIM2_CH2_Write(uint16_t val) { HAL_PWM_SetCompare(TIM2, 2, val); }


// TIM4 Channels (PB6 -> CH1, PB7 -> CH2)
static void TIM4_CH1_Write(uint16_t val) { HAL_PWM_SetCompare(TIM4, 1, val); }
static void TIM4_CH2_Write(uint16_t val) { HAL_PWM_SetCompare(TIM4, 2, val); }

// TIM3 Channels
static void TIM3_CH1_Write(uint16_t val) { HAL_PWM_SetCompare(TIM3, 1, val); }
static void TIM3_CH2_Write(uint16_t val) { HAL_PWM_SetCompare(TIM3, 2, val); }
static void TIM3_CH3_Write(uint16_t val) { HAL_PWM_SetCompare(TIM3, 3, val); }
static void TIM3_CH4_Write(uint16_t val) { HAL_PWM_SetCompare(TIM3, 4, val); }

// ==========================================
// 2. 资源实例化 (Handles)
// ==========================================
static Servo_Handle_t hServo_LT_Knee; // Left Top Knee
static Servo_Handle_t hServo_LT_Hip;  // Left Top Hip
static Servo_Handle_t hServo_RT_Knee; // Right Top Knee
static Servo_Handle_t hServo_RT_Hip;  // Right Top Hip
static Servo_Handle_t hServo_RB_Knee; // Right Bottom Knee
static Servo_Handle_t hServo_RB_Hip;  // Right Bottom Hip
static Servo_Handle_t hServo_LB_Knee; // Left Bottom Knee
static Servo_Handle_t hServo_LB_Hip;  // Left Bottom Hip

// ==========================================
// 3. 初始化
// ==========================================
void BSP_Servo_Init(void)
{
    // 1. 初始化 GPIO (复用推挽)
    // PA0-3 (TIM2), PA6-7 (TIM3_CH1/2)
    HAL_GPIO_Init(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7, HAL_GPIO_MODE_AF_PP);
		// PB0-1 (TIM3_CH3/4)
    HAL_GPIO_Init(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7, HAL_GPIO_MODE_AF_PP);
	
		// 2. 初始化 PWM 定时器
    HAL_PWM_TimerInit(TIM2);
    HAL_PWM_TimerInit(TIM3);
    HAL_PWM_TimerInit(TIM4); 
    
    // 3. 配置所有通道
    for(int i=1; i<=4; i++) {
        HAL_PWM_ConfigChannel(TIM3, i); // TIM3 依然全开4通道
    }
    HAL_PWM_ConfigChannel(TIM2, 1);
    HAL_PWM_ConfigChannel(TIM2, 2);
    HAL_PWM_ConfigChannel(TIM4, 1); // TIM4_CH1对应PB6
    HAL_PWM_ConfigChannel(TIM4, 2); // TIM4_CH2对应PB7

    // 4. 绑定句柄到具体的硬件通道 (根据原代码逻辑映射)
    Dev_Servo_Init(&hServo_LT_Knee, TIM3_CH1_Write);
    Dev_Servo_Init(&hServo_LT_Hip,  TIM3_CH2_Write);
    Dev_Servo_Init(&hServo_RT_Knee, TIM2_CH1_Write);
    Dev_Servo_Init(&hServo_RT_Hip,  TIM2_CH2_Write);
		Dev_Servo_Init(&hServo_RB_Knee, TIM4_CH2_Write); 
    Dev_Servo_Init(&hServo_RB_Hip,  TIM4_CH1_Write); 
    Dev_Servo_Init(&hServo_LB_Knee, TIM3_CH4_Write);
    Dev_Servo_Init(&hServo_LB_Hip,  TIM3_CH3_Write);
}

// ==========================================
// 4. 应用接口实现
// ==========================================
void BSP_Servo_Set_Left_Top_Knee(float angle)   { Dev_Servo_SetAngle(&hServo_LT_Knee, angle); }
void BSP_Servo_Set_Left_Top_Hip(float angle)    { Dev_Servo_SetAngle(&hServo_LT_Hip, angle); }
void BSP_Servo_Set_Right_Top_Knee(float angle)  { Dev_Servo_SetAngle(&hServo_RT_Knee, angle); }
void BSP_Servo_Set_Right_Top_Hip(float angle)   { Dev_Servo_SetAngle(&hServo_RT_Hip, angle); }
void BSP_Servo_Set_Right_Bottom_Knee(float angle){ Dev_Servo_SetAngle(&hServo_RB_Knee, angle); }
void BSP_Servo_Set_Right_Bottom_Hip(float angle) { Dev_Servo_SetAngle(&hServo_RB_Hip, angle); }
void BSP_Servo_Set_Left_Bottom_Knee(float angle) { Dev_Servo_SetAngle(&hServo_LB_Knee, angle); }
void BSP_Servo_Set_Left_Bottom_Hip(float angle)  { Dev_Servo_SetAngle(&hServo_LB_Hip, angle); }
