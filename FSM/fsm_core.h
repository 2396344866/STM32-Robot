#ifndef FSM_CORE_H
#define FSM_CORE_H
/**
 * @file fsm_core.h
 * @brief 有限状态机核心模块接口定义
 * 
 * 该模块提供了一个轻量级、事件驱动的有限状态机框架。
 * 主要特性：
 * - 基于事件队列的异步事件处理
 * - 支持状态进入/退出/轮询回调
 * - 支持带守卫条件的迁移动作
 * - 环形缓冲区实现的事件队列
 * - 可选的调试信息支持
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sys_config.h"
// 类型定义
typedef int16_t fsm_state_t;
/** 
 * @brief 表示无效状态
 * 
 * 用于初始化或错误处理场景，表示当前没有有效状态
 */
#define FSM_STATE_NONE  -1

struct fsm_t; // 前向声明

/**
 * @brief 事件结构体
 * 
 * 定义了状态机处理的基本事件单元
 */
typedef struct {
    uint16_t id;      /**< 事件ID，用于在迁移表中匹配事件 */
    uint32_t param;   /**< 事件参数，传递给动作回调函数 */
} fsm_event_t;


/**
 * @brief 状态机动作回调函数原型
 * 
 * 用于状态进入、退出、轮询以及迁移动作的回调函数类型。
 * 该回调在状态机上下文中执行，应避免长时间阻塞。
 * 
 * @param fsm        触发回调的状态机实例指针
 * @param user_data  用户自定义数据指针（通过fsm_init传入）
 * 
 * @note 回调函数中不应修改状态机内部数据结构（如直接推送事件除外）
 * @note 如果需要在回调中推送事件，应使用fsm_push_event
 */
typedef void (*fsm_action_cb)(struct fsm_t* fsm, void* user_data);

/**
 * @brief 状态机守卫条件回调函数原型
 * 
 * 用于迁移表中的守卫条件判断。决定在当前条件下是否允许执行迁移。
 * 该回调在状态机处理事件时同步调用。
 * 
 * @param fsm        触发回调的状态机实例指针
 * @param user_data  用户自定义数据指针（通过fsm_init传入）
 * @return true      允许执行迁移
 * @return false     禁止执行迁移（继续查找其他匹配规则）
 * 
 * @note 守卫回调应保持轻量，避免复杂计算或阻塞操作
 * @note 守卫回调不应修改状态机状态或外部系统状态（应为纯判断函数）
 */
typedef bool (*fsm_guard_cb)(struct fsm_t* fsm, void* user_data);
/**
 * @brief 状态描述
 * 
 * 定义某个状态的行为回调函数
 * 
 * @note 这些回调函数在特定时机被调用，不应包含阻塞操作
 */
typedef struct {
    fsm_state_t     state_id;    /**< 关联的状态ID */
    fsm_action_cb   on_enter;    /**< 进入状态时的回调（状态激活时调用一次） */
    fsm_action_cb   on_exit;     /**< 退出状态时的回调（状态离开时调用一次） */
    fsm_action_cb   on_poll;     /**< 轮询回调（每次fsm_run调用时执行，非事件触发） */
} fsm_state_desc_t;
/**
 * @brief 状态迁移表项
 * 
 * 定义了在特定状态下收到特定事件时应该执行的迁移规则
 */
typedef struct {
    fsm_state_t     curr_state;   /**< 源状态 */
    uint16_t        evt_id;       /**< 触发事件ID */
    fsm_state_t     next_state;   /**< 目标状态 */
    fsm_guard_cb    guard;        /**< 守卫条件（返回true表示允许迁移，可置NULL） */
    fsm_action_cb   trans_action; /**< 迁移动作（迁移过程中执行，可置NULL） */
} fsm_transition_t;
/**
 * @brief 调试信息结构体
 * 
 * 提供状态和事件名称的字符串映射，用于调试输出
 */
typedef struct {
    const char* const* state_names;  /**< 状态ID到状态名称的映射数组 */
    const char* const* event_names;  /**< 事件ID到事件名称的映射数组 */
} fsm_debug_info_t;
/**
 * @brief 状态机句柄
 * 
 * 包含状态机的所有运行时数据
 * 
 * @note volatile修饰的成员可能在中断中被修改
 */
typedef struct fsm_t {
    volatile fsm_state_t current_state;   /**< 当前状态 */
    void* user_data;                       /**< 用户自定义数据指针（透传给回调） */
    uint32_t current_param;                 /**< 当前处理事件的参数（供回调使用） */

    // 事件队列（环形缓冲区）
    fsm_event_t* evt_queue_buff;            /**< 事件队列缓冲区 */
    uint16_t     queue_size;                 /**< 缓冲区大小（元素个数） */
    volatile uint16_t head;                   /**< 队列头部索引（写入位置） */
    volatile uint16_t tail;                   /**< 队列尾部索引（读取位置） */
    
    // 配置表
    const fsm_transition_t* trans_table;    /**< 状态迁移表 */
    size_t                  trans_count;     /**< 迁移表条目数 */
    const fsm_state_desc_t* state_desc;      /**< 状态描述表 */
    size_t                  desc_count;      /**< 状态描述表条目数 */
    const fsm_debug_info_t* debug_info;      /**< 调试信息（可NULL） */
} fsm_t;

// API
/**
 * @brief 初始化状态机实例
 * 
 * 在使用状态机之前必须先调用此函数进行初始化。
 * 事件队列缓冲区由调用者提供，可以是静态数组或动态分配的内存。
 * 
 * @param fsm           状态机指针（非空）
 * @param evt_buff      事件队列缓冲区（非空）
 * @param buff_size     缓冲区大小（元素个数，必须>0）
 * @param trans_table   状态迁移表（非空）
 * @param trans_count   迁移表条目数
 * @param init_state    初始状态
 * @param user_data     用户自定义数据指针（透传给所有回调）
 * 
 * @note 如果任何必需指针为NULL，函数将直接返回而不执行初始化
 */
void fsm_init(fsm_t* fsm, fsm_event_t* evt_buff, uint16_t buff_size,
              const fsm_transition_t* trans_table, size_t trans_count,
              fsm_state_t init_state, void* user_data);

/**
 * @brief 设置状态描述表（包含各状态的进入/退出/轮询回调）
 * 
 * 此函数在初始化后调用，为状态机注册状态的行为回调。
 * 如果不设置状态描述表，状态机仍可运行，但不会执行任何状态回调。
 * 
 * @param fsm       状态机指针
 * @param desc      状态描述符数组
 * @param count     数组元素个数
 * 
 * @note 状态描述表在状态机运行期间必须保持有效
 */
void fsm_set_state_callbacks(fsm_t* fsm, const fsm_state_desc_t* desc, size_t count);

/**
 * @brief 设置调试信息结构体
 * 
 * 提供状态和事件的名称映射，用于调试输出。
 * 此功能是可选的，如果不设置，未处理事件时不会输出调试信息。
 * 
 * @param fsm           状态机指针
 * @param debug_info    调试信息指针
 * 
 * @note 调试信息结构体在状态机运行期间必须保持有效
 */
void fsm_set_debug_info(fsm_t* fsm, const fsm_debug_info_t* debug_info);

/**
 * @brief 向状态机事件队列推送一个事件
 * 
 * 将事件放入环形缓冲区，供fsm_run处理。
 * 此函数是线程安全的（受临界区保护），可在中断上下文中调用。
 * 
 * @param fsm       状态机指针
 * @param evt_id    事件ID
 * @param param     事件参数
 * @return true     推送成功
 * @return false    队列已满或fsm指针为NULL
 * 
 * @note 当队列满时，事件会被丢弃并返回false
 */
bool fsm_push_event(fsm_t* fsm, uint16_t evt_id, uint32_t param);

/**
 * @brief 运行状态机（轮询 + 事件处理）
 * 
 * 该函数执行两个主要任务：
 * 1. 调用当前状态的轮询回调（on_poll）
 * 2. 循环处理事件队列中的所有事件，直到队列为空
 * 
 * 事件处理流程：
 * - 从队列取出事件
 * - 遍历迁移表查找匹配规则
 * - 执行守卫条件检查
 * - 执行状态切换（调用on_exit/trans_action/on_enter）
 * - 如果未找到匹配规则且启用了调试信息，输出错误
 * 
 * @param fsm 状态机指针
 * 
 * @note 此函数不应在中断上下文中调用
 */
void fsm_run(fsm_t* fsm);

#endif
