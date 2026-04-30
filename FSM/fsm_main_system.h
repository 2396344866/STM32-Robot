#ifndef FSM_MAIN_SYSTEM_H
#define FSM_MAIN_SYSTEM_H

#include "fsm_core.h"
#include "timers.h"
#include "sys_events.h"
//#include "fsm_motor.h" 
#include "sys_config.h"
typedef enum {
    STATE_INIT = 0,
		STATE_LINKING,
    STATE_IDLE,
    STATE_MOTOR_CTRL,
		STATE_BLOCKED,      // <-- 新增：避障锁定状态
    STATE_ERROR,
    Main_System_STATE_MAX
} Main_System_state_t;

typedef struct {
    uint8_t       current_selection;
    TimerHandle_t xMenuTimer;
    uint8_t       m1_menu_index;     // 当前选中的电机动作
		uint8_t run_source;     // 运行源：0=停止, 1=本地按键, 2=APP云端
} Main_System_context_t;

void Main_System_fsm_setup(fsm_t* fsm);
void Main_System_FSM_task(void *pvParameters);
#endif
