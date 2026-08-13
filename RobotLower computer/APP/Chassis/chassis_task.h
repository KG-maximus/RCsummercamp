#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "chassis_control.h"

typedef struct
{
    ChassisControl_Status_t control;          /* 控制器状态机和轮速快照。 */
    ChassisControl_Result_t last_command_result; /* 最近一条命令的接受/拒绝结果。 */
    uint8_t configuration_ready;               /* chassis_config 参数是否完成控制器初始化。 */
    uint8_t port_ready;                        /* 四个电调是否持续有新鲜反馈。 */
    uint8_t rtos_objects_ready;                /* 命令队列和状态 Mutex 是否创建成功。 */
} ChassisTask_Status_t;

/*
 * 创建命令消息队列、状态互斥锁并初始化 M3508 Port/控制器。
 * 必须在 osKernelInitialize() 之后、osKernelStart() 之前调用。当前配置未
 * 就绪时函数仍可成功返回，供调试任务读取状态；configuration_ready 会是 0U，
 * 控制线程不会接受运动命令。
 */
uint8_t ChassisTask_Init(void);

/*
 * 底盘固定周期任务入口，由 freertos.c 创建 CMSIS-RTOS v2 线程后调用。
 * 任务依次读取异步反馈、取尽队列命令、执行一次控制、下发轮速并发布状态，
 * 最后以 osDelayUntil 保持 CHASSIS_CONTROL_PERIOD_MS 的稳定节拍。
 */
void ChassisTask_Entry(void *argument);

/*
 * 供上位机解析任务、遥控任务或机器人总状态机投递一条 ChassisCommand_t。
 * timeout_ms 是等待队列空位的最长时间，单位 ms；中断回调不可调用本函数。
 * ESTOP 优先级最高，DISABLE/STOP 次之。返回 1U 表示已入队，不代表命令已
 * 通过控制器的格式、状态机和硬件安全校验。来源仲裁、序号去重和业务有效期
 * 应由上位机通信任务或机器人总状态机在调用本接口前完成。对上状态接口为
 * ChassisTask_GetStatus()；对下电机接口封装在 ChassisPort_*()，本任务负责
 * 在两者之间调度，不应由通信模块绕过本任务直接驱动电机。
 */
uint8_t ChassisTask_PostCommand(
    const ChassisCommand_t *command,
    uint32_t timeout_ms);

/*
 * 复制带 Mutex 保护的一致性状态快照，适用于日志、上位机回传和调试显示。
 * 调用会最多等待约 10 ms 获取 Mutex；返回 0U 时调用者应放弃本次读取，不能
 * 直接访问 chassis_task.c 内的静态变量。
 */
uint8_t ChassisTask_GetStatus(ChassisTask_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
