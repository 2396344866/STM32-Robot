#include "dev_servo.h"

void Dev_Servo_Init(Servo_Handle_t* hServo, Servo_WritePWM_Func write_func)
{
    hServo->SetPWM = write_func;
    hServo->min_angle = 0.0f;
    hServo->max_angle = 180.0f;
}

void Dev_Servo_SetAngle(Servo_Handle_t* hServo, float angle)
{
    if (!hServo->SetPWM) return;

    // 限制角度范围
    if (angle < hServo->min_angle) angle = hServo->min_angle;
    if (angle > hServo->max_angle) angle = hServo->max_angle;

    // 核心算法：0~180 -> 500~2500 (对应 0.5ms ~ 2.5ms)
    // 假设 Timer Period = 20000 (20ms)
    uint16_t pwm_val = (uint16_t)(angle / 180.0f * 2000.0f + 500.0f);
    
    hServo->SetPWM(pwm_val);
}
