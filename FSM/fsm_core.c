/**
 * @file fsm_core.c
 * @brief 有限状态机核心实现
 * 
 * 该模块提供了一个基于事件队列和状态迁移表的状态机框架。
 * 支持状态进入/退出/轮询回调、带守卫条件的迁移动作以及调试信息输出。
 */
#include "fsm_core.h"
#include <stdio.h>
/**
 * @brief 初始化状态机实例
 * 
 * @param fsm           状态机指针（非空）
 * @param evt_buff      事件队列缓冲区（非空）
 * @param buff_size     缓冲区大小（元素个数）
 * @param trans_table   状态迁移表（非空）
 * @param trans_count   迁移表条目数
 * @param init_state    初始状态
 * @param user_data     用户自定义数据指针
 */
void fsm_init(fsm_t* fsm, fsm_event_t* evt_buff, uint16_t buff_size,
              const fsm_transition_t* trans_table, size_t trans_count,
              fsm_state_t init_state, void* user_data) 
{
    if (!fsm || !evt_buff || !trans_table) return;

    fsm->evt_queue_buff = evt_buff;
    fsm->queue_size = buff_size;
    fsm->head = 0;
    fsm->tail = 0;
    fsm->trans_table = trans_table;
    fsm->trans_count = trans_count;
    fsm->current_state = init_state;
    fsm->user_data = user_data;
    fsm->state_desc = NULL;
    fsm->debug_info = NULL;
}
/**
 * @brief 设置状态描述表（包含各状态的进入/退出/轮询回调）
 * 
 * @param fsm       状态机指针
 * @param desc      状态描述符数组
 * @param count     数组元素个数
 */
void fsm_set_state_callbacks(fsm_t* fsm, const fsm_state_desc_t* desc, size_t count) {
    if(fsm) {
        fsm->state_desc = desc;
        fsm->desc_count = count;
    }
}
/**
 * @brief 设置调试信息结构体（用于错误报告）
 * 
 * @param fsm           状态机指针
 * @param debug_info    调试信息指针
 */
void fsm_set_debug_info(fsm_t* fsm, const fsm_debug_info_t* debug_info) {
    if(fsm) fsm->debug_info = debug_info;
}
/**
 * @brief 向状态机事件队列推送一个事件（线程安全）
 * 
 * @param fsm       状态机指针
 * @param evt_id    事件ID
 * @param param     事件参数
 * @return true     推送成功
 * @return false    队列已满或参数无效
 */
bool fsm_push_event(fsm_t* fsm, uint16_t evt_id, uint32_t param) {
    if (!fsm) return false;
    
    FSM_ENTER_CRITICAL();          // 进入临界区，防止中断或多线程竞争
    
    // 计算下一个头部位置（环形缓冲区）
    uint16_t next_head = (fsm->head + 1) % fsm->queue_size;
    
    // 如果下一个头部位置等于尾部，说明队列已满
    if (next_head == fsm->tail) {
        FSM_EXIT_CRITICAL();
        return false;               // 队列满，事件丢失
    }
    
    // 在当前位置存入事件
    fsm->evt_queue_buff[fsm->head].id = evt_id;
    fsm->evt_queue_buff[fsm->head].param = param;
    fsm->head = next_head;          // 更新头部指针
    
    FSM_EXIT_CRITICAL();            // 退出临界区
    return true;
}
/**
 * @brief 根据状态ID查找对应的状态描述符
 * 
 * @param fsm       状态机指针
 * @param state     要查找的状态ID
 * @return const fsm_state_desc_t* 找到的描述符指针，未找到返回NULL
 */
static const fsm_state_desc_t* find_desc(fsm_t* fsm, fsm_state_t state) {
    if (!fsm->state_desc) return NULL;
    for (size_t i = 0; i < fsm->desc_count; i++) {
        if (fsm->state_desc[i].state_id == state) {
            return &fsm->state_desc[i];
        }
    }
    return NULL;
}
/**
 * @brief 运行状态机（轮询 + 事件处理）
 * 
 * 该函数执行两个主要任务：
 * 1. 调用当前状态的轮询回调
 * 2. 循环处理事件队列中的所有事件，直到队列为空
 * 
 * @param fsm 状态机指针
 */
void fsm_run(fsm_t* fsm) {
    if (!fsm) return;

    // 1. 执行轮询操作：先让当前状态有机会执行持续性的动作
    const fsm_state_desc_t* curr_desc = find_desc(fsm, fsm->current_state);
    if (curr_desc && curr_desc->on_poll) {
        curr_desc->on_poll(fsm, fsm->user_data);
    }

    // 2. 处理事件队列：一次性处理所有待处理事件
    while (1) {
        fsm_event_t evt;
        
        // 从队列中取出一个事件（线程安全）
        FSM_ENTER_CRITICAL();
        if (fsm->head == fsm->tail) {
            FSM_EXIT_CRITICAL();    // 队列为空，退出循环
            break;
        }
        evt = fsm->evt_queue_buff[fsm->tail];   // 从尾部取出事件
        fsm->tail = (fsm->tail + 1) % fsm->queue_size; // 更新尾部指针
        FSM_EXIT_CRITICAL();

        // 保存当前事件参数，供回调函数使用
        fsm->current_param = evt.param;

        bool transitioned = false;
        
        // 遍历迁移表，查找匹配的迁移规则
        for (size_t i = 0; i < fsm->trans_count; i++) {
            const fsm_transition_t* tr = &fsm->trans_table[i];
            
            // 匹配条件：当前状态相同 且 事件ID相同
            if (tr->curr_state == fsm->current_state && tr->evt_id == evt.id) {
                // 如果有守卫条件，且条件不满足，则跳过此规则
                if (tr->guard && !tr->guard(fsm, fsm->user_data)) continue;

                fsm_state_t next = tr->next_state;
                
                if (next != fsm->current_state) {
                    // 情况1：状态切换（目标状态与当前不同）
                    
                    // 先执行当前状态的退出回调
                    if (curr_desc && curr_desc->on_exit) 
                        curr_desc->on_exit(fsm, fsm->user_data);
                    
                    // 执行迁移动作（切换时的公共动作）
                    if (tr->trans_action) 
                        tr->  trans_action(fsm, fsm->user_data);
                    
                    // 更新当前状态
                    fsm->current_state = next;
                    
                    // 查找新状态的描述符，并执行其进入回调
                    const fsm_state_desc_t* next_desc = find_desc(fsm, next);
                    if (next_desc && next_desc->on_enter) 
                        next_desc->on_enter(fsm, fsm->user_data);
                    
                    // 更新当前描述符指针，供下一轮使用
                    curr_desc = next_desc;
                } else {
                    // 情况2：自流转（目标状态等于当前状态）
                    // 只执行迁移动作，不触发进入/退出回调
                    if (tr->trans_action) 
                        tr->trans_action(fsm, fsm->user_data);
                }
                
                transitioned = true;
                break;  // 找到匹配规则，停止遍历
            }
        }
        
        // 如果没有找到任何匹配的迁移规则，且启用了调试信息，则输出错误
        if (!transitioned && fsm->debug_info) {
            printf("Unhandled evt %d in state %d\n", evt.id, fsm->current_state);
        }
    }
}
