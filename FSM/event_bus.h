/**
 * @file event_bus.h
 * @brief 简单事件总线模块接口
 * 
 * 该模块实现了一个基于订阅-发布模式的事件总线。
 * 状态机可以通过订阅机制注册感兴趣的事件，当事件发布时，
 * 总线会自动将事件推送给所有订阅了该事件的状态机。
 * 
 * @note 本模块依赖于fsm_core模块的fsm_push_event函数
 */

#ifndef EVENT_BUS_H
#define EVENT_BUS_H

#include <stdint.h>
#include "fsm_core.h"

/**
 * @brief 初始化事件总线
 * 
 * 在使用事件总线之前必须调用此函数进行初始化。
 * 该函数会清空所有订阅记录。
 */
void event_bus_init(void);

/**
 * @brief 订阅事件
 * 
 * 状态机调用此函数注册对特定事件的关注。
 * 当该事件被发布时，总线会自动将事件推送给该状态机。
 * 
 * @param fsm       订阅者状态机指针
 * @param event_id  要订阅的事件ID
 * 
 * @note 订阅表大小有限（由BUS_MAX_SUBS定义），订阅者过多时会失败（静默丢弃）
 * @note 同一个状态机可以订阅多个不同的事件
 */
void event_bus_subscribe(fsm_t* fsm, uint16_t event_id);

/**
 * @brief 发布事件
 * 
 * 向总线广播一个事件。所有订阅了该事件ID的状态机
 * 都会通过fsm_push_event收到该事件。
 * 
 * @param event_id  要发布的事件ID
 * @param param     事件参数（透传给接收者）
 * 
 * @note 此函数可以在中断上下文中调用（因为它调用fsm_push_event）
 * @note 如果订阅者的事件队列已满，事件可能会丢失
 */
void event_bus_publish(uint16_t event_id, uint32_t param);

#endif
