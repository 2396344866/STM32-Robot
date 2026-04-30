/**
 * @file event_bus.c
 * @brief 事件总线模块实现
 * 
 * 采用静态数组实现的简单订阅表，不支持动态内存分配。
 * 适用于资源受限的嵌入式系统。
 */

#include "event_bus.h"
#include <stddef.h>
/**
 * @brief 订阅记录结构
 * 
 * 保存状态机与其订阅的事件ID的对应关系
 */
typedef struct {
    fsm_t* subscriber_fsm; /**< 订阅者状态机指针 */
    uint16_t event_id;      /**< 订阅的事件ID */
} bus_record_t;

/** @brief 静态订阅表 */
static bus_record_t sub_table[BUS_MAX_SUBS];

/** @brief 当前已使用的订阅记录数 */
static uint8_t      sub_count = 0;
/**
 * @brief 初始化事件总线
 * 
 * 将订阅计数器清零，清空所有订阅记录。
 */
void event_bus_init(void) { 
    sub_count = 0; 
}
/**
 * @brief 订阅事件
 * 
 * 在订阅表中添加一条新的订阅记录。
 * 如果订阅表已满，则静默失败（不添加）。
 * 
 * @param fsm       订阅者状态机指针
 * @param event_id  要订阅的事件ID
 */
void event_bus_subscribe(fsm_t* fsm, uint16_t event_id) {
    // 检查订阅表是否还有空闲位置
    if (sub_count < BUS_MAX_SUBS) {
        sub_table[sub_count].subscriber_fsm = fsm;
        sub_table[sub_count].event_id = event_id;
        sub_count++;
    }
    // 订阅表已满时静默失败（可在此添加调试输出）
}

/**
 * @brief 发布事件
 * 
 * 遍历订阅表，向所有订阅了指定事件ID的状态机推送事件。
 * 使用fsm_push_event将事件送入各状态机的事件队列。
 * 
 * @param event_id  要发布的事件ID
 * @param param     事件参数
 * 
 * @note 采用线性查找，订阅者较多时可能影响性能
 */
void event_bus_publish(uint16_t event_id, uint32_t param) {
    // 遍历所有订阅记录
    for (uint8_t i = 0; i < sub_count; i++) {
        // 如果记录的事件ID匹配
        if (sub_table[i].event_id == event_id) {
            // 向订阅者推送事件
            fsm_push_event(sub_table[i].subscriber_fsm, event_id, param);
        }
    }
}
