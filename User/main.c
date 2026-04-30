#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
/* --- HAL层 ---*/
#include "HAL_delay.h"
/* --- BSP层 ---*/
#include "bsp_LED.h"
#include "bsp_debug_uart.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_mpu6050.h"
#include "bsp_esp8266.h"
/* --- 业务解耦模块 --- */
#include "sys_events.h"
#include "fsm_core.h"
#include "fsm_key.h"         // 包含用户按键输入（切换状态）
#include "fsm_motor.h"       // 包含 Motor_FSM_Setup
#include "fsm_main_system.h" // 包含 Main_System_fsm_setup
#include "fsm_network.h"			// 包含 Network_FSM_Setup
#include "fsm_sensor.h"
#include "event_bus.h"       // 用于总线初始化
#include "sys_config.h"
/* ================= 任务配置 ================= */
#define START_TASK_PRIO         1
#define START_STK_SIZE          512
#define MOTOR_TASK_PRIO         5
#define MOTOR_STK_SIZE          256
#define MAIN_SYS_TASK_PRIO      3
#define MAIN_SYS_STK_SIZE       256  
#define NETWORK_TASK_PRIO       4  
#define NETWORK_STK_SIZE        512 
#define SENSOR_TASK_PRIO   2 
#define SENSOR_STK_SIZE    256
TaskHandle_t StartTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t MainSysTask_Handler;
TaskHandle_t NetworkTask_Handler;
TaskHandle_t SensorTask_Handler;
// 全局按键逻辑队列 (跨模块通信用，属于 OS 级资源)
QueueHandle_t xKeyLogicQueue;
void start_task(void *pvParameters);
//// 全局 FSM 句柄
//fsm_t g_Main_System_fsm;
//// 按键逻辑队列
//QueueHandle_t xKeyLogicQueue;
//// 全局存放姿态数据的结构体
//MPU6050_Data_t g_imu_data;
//// 任务函数声明
//void start_task(void *pvParameters);
//void Motor_FSM_task(void *pvParameters);
//void Main_System_FSM_task(void *pvParameters);
//void Network_FSM_Task(void *pvParameters);

int main()
{
		// 硬件层初始化 (必须在调度器启动前完成)
    // 确保你的中断分组设置为 Group 4 (FreeRTOS 推荐)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		HAL_Delay_Init();
		BSP_LED_Init();	
		
		//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度，创建一个空闲任务（IDLE Task）							
		while(1){};
}

void start_task(void *pvParameters){
    // 1. 进入临界区，保护创建过程
		taskENTER_CRITICAL();           
    // --- A. 资源分配 (优先级最高) ---
    // 创建 Key 事件队列
    xKeyLogicQueue = xQueueCreate(10, sizeof(sys_event_t));
    
    // 严格的错误检查：如果核心队列创建失败，系统不能启动
    if(xKeyLogicQueue == NULL) {
        printf("Queue Creation Failed! System Halted.\r\n");
        // 陷入死循环或触发硬件复位，防止后续空指针操作
        while(1) {}; 
    }
    // 初始化总线，清空订阅表
    event_bus_init();
		
		
		// --- B. 中断硬件初始化(硬件上电死机大概率就是提前开启中断 中断触发导致)---
    // 必须在 xKeyLogicQueue 创建成功  后调用，否则中断中写队列会 HardFault
		KEY_Init();
		#if ENABLE_DEBUG_PRINT
				BSP_DebugUART_Init(115200); /* 调试模式：只初始化串口 */
				printf("BSP_DebugUART_Init Complete\n");
		#endif
		BSP_Servo_Init();           /* 运行模式：只初始化舵机 */
		BSP_ESP8266_Init(115200);
    printf("BSP_ESP8266_Init Complete\n");
		BSP_OLED_Init();
		BSP_OLED_ShowString(1,1,"OK");
		printf("BSP_OLED_Init\n");
		BSP_MPU6050_Init(); 
		printf("BSP_MPU6050_Init\n");
		
		
		
    // --- C. 创建业务任务 ---
    // 电机任务
    xTaskCreate((TaskFunction_t )Motor_FSM_task,
                (const char* )"Motor_Task",
                (uint16_t       )MOTOR_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )MOTOR_TASK_PRIO,
                (TaskHandle_t* )&MotorTask_Handler);
    // 主系统任务
    xTaskCreate((TaskFunction_t )Main_System_FSM_task,
                (const char* )"Main_Task",
                (uint16_t       )MAIN_SYS_STK_SIZE,
                (void* )NULL,
                (UBaseType_t    )MAIN_SYS_TASK_PRIO,
                (TaskHandle_t* )&MainSysTask_Handler);
		// 网络处理任务
    xTaskCreate((TaskFunction_t )Network_FSM_Task,
                (const char* )"Net_Task",
                (uint16_t    )NETWORK_STK_SIZE,
                (void* )NULL,
                (UBaseType_t )NETWORK_TASK_PRIO,
                (TaskHandle_t* )&NetworkTask_Handler);
		// 传感器轮询 ---
    xTaskCreate((TaskFunction_t )Sensor_FSM_Task,
                (const char* )"Sensor_Task",
                (uint16_t    )SENSOR_STK_SIZE,
                (void* )NULL,
                (UBaseType_t )SENSOR_TASK_PRIO,
                (TaskHandle_t* )&SensorTask_Handler);						
    // --- D. 退出与清理 ---
    // 退出临界区-恢复中断
    taskEXIT_CRITICAL();            
    // 最后删除创建任务函数
    vTaskDelete(StartTask_Handler); 
}
