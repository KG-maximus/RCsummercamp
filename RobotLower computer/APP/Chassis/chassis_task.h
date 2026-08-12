#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "chassis_control.h"

typedef struct
{
    ChassisControl_Status_t control;
    ChassisControl_Result_t last_command_result;
    uint8_t configuration_ready;
    uint8_t port_ready;
    uint8_t rtos_objects_ready;
} ChassisTask_Status_t;

/* 在 osKernelInitialize() 之后、osKernelStart() 之前调用。 */
uint8_t ChassisTask_Init(void);

/* 由 freertos.c 创建线程并作为任务入口。 */
void ChassisTask_Entry(void *argument);

/* 上位机解析任务或机器人总状态机使用该接口投递命令。 */
uint8_t ChassisTask_PostCommand(
    const ChassisCommand_t *command,
    uint32_t timeout_ms);

/* 上位机回传/调试任务可周期读取状态快照。 */
uint8_t ChassisTask_GetStatus(ChassisTask_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
