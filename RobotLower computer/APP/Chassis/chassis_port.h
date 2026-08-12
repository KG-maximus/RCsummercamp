#ifndef CHASSIS_PORT_H
#define CHASSIS_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "chassis_mecanum.h"

/*
 * 底盘算法与具体电机/FDCAN 协议之间的唯一边界。
 * 当前实现已对接 C620 + M3508。运动学仍然只看 FL/FR/RL/RR rpm，
 * CAN ID、反馈解析和速度 PID 仍封装在 chassis_port.c 和 APP/Common/M3508.c。
 */
uint8_t ChassisPort_Init(void);
uint8_t ChassisPort_IsReady(void);

uint8_t ChassisPort_SendMotorRpm(
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]);

uint8_t ChassisPort_ReadMotorRpm(
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t *feedback_time_ms);

void ChassisPort_SetMotorEnabled(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
