#ifndef __DEV_SERVO_H
#define __DEV_SERVO_H

#include <stdint.h>

// 定义一个函数指针类型，用于底层写入 PWM 比较值
// 这样 dev_servo 就不需要包含 stm32f10x.h，也不绑定特定 Timer
typedef void (*Servo_WritePWM_Func)(uint16_t compare_val);

// 舵机句柄
typedef struct {
    Servo_WritePWM_Func SetPWM; // 依赖注入接口
    float min_angle;            // 机械限位 (例如 0)
    float max_angle;            // 机械限位 (例如 180)
} Servo_Handle_t;

// 初始化
void Dev_Servo_Init(Servo_Handle_t* hServo, Servo_WritePWM_Func write_func);
// 设置角度
void Dev_Servo_SetAngle(Servo_Handle_t* hServo, float angle);

#endif
