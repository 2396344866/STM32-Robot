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
    uint8_t       current_selection;    //目前的选择
    TimerHandle_t xMenuTimer;          //UI界面定时器
    uint8_t       m1_menu_index;     // 当前选中的电机动作
		uint8_t run_source;     // 运行源：0=没有来源, 1=本地按键操作, 2=APP云端控制
} Main_System_context_t;

void Main_System_fsm_setup(fsm_t* fsm);
void Main_System_FSM_task(void *pvParameters);
#endif
