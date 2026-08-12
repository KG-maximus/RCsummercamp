#include "chassis_port.h"

#include <string.h>

#include "ControlAlgorithm.h"
#include "M3508.h"
#include "bsp_callback.h"
#include "chassis_config.h"
#include "main.h"

/*
 * This adapter is the only place where chassis FL/FR/RL/RR wheel order meets
 * the C620 protocol IDs. The kinematics module stays independent of CAN.
 */
static M3508Group_TypeDef chassis_motor_group;
static uint8_t chassis_port_initialized;
static uint8_t chassis_motor_enabled;

static const uint8_t chassis_motor_id[CHASSIS_MECANUM_WHEEL_COUNT] =
{
    CHASSIS_M3508_ID_FL,
    CHASSIS_M3508_ID_FR,
    CHASSIS_M3508_ID_RL,
    CHASSIS_M3508_ID_RR
};

static uint8_t ChassisPort_IsElapsed(
    uint32_t now_ms,
    uint32_t timestamp_ms,
    uint32_t timeout_ms)
{
    return (uint8_t)((timestamp_ms == 0U) ||
                     ((uint32_t)(now_ms - timestamp_ms) > timeout_ms));
}

static void ChassisPort_FDCANRxHandler(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint32_t std_id,
    const uint8_t data[8])
{
    if (fdcan_handle != CHASSIS_M3508_CAN_HANDLE)
    {
        return;
    }

    (void)M3508GroupParseFeedback(
        &chassis_motor_group,
        std_id,
        data,
        HAL_GetTick());
}

uint8_t ChassisPort_Init(void)
{
    CascadePID_TypeDef pid_template;

    if (chassis_port_initialized != 0U)
    {
        return ChassisPort_IsReady();
    }

    CascadePIDInit(
        &pid_template,
        CHASSIS_M3508_SPEED_KP,
        CHASSIS_M3508_SPEED_KI,
        CHASSIS_M3508_SPEED_KD,
        CHASSIS_M3508_SPEED_MAX_OUT,
        CHASSIS_M3508_SPEED_MAX_IOUT,
        CHASSIS_M3508_CURRENT_KP,
        CHASSIS_M3508_CURRENT_KI,
        CHASSIS_M3508_CURRENT_KD,
        CHASSIS_M3508_CURRENT_MAX_OUT,
        CHASSIS_M3508_CURRENT_MAX_IOUT);
    M3508GroupInit(
        &chassis_motor_group,
        CHASSIS_M3508_CAN_HANDLE,
        CHASSIS_M3508_CONTROL_ID,
        &pid_template);
    if (BSPCallback_RegisterFDCANRxHandler(ChassisPort_FDCANRxHandler) == 0U)
    {
        return 0U;
    }
    FDCANStandardInit(
        CHASSIS_M3508_CAN_HANDLE,
        M3508_FEEDBACK_ID_BASE + CHASSIS_M3508_ID_FL,
        M3508_FEEDBACK_ID_BASE + CHASSIS_M3508_ID_RR);

    chassis_motor_enabled = 0U;
    chassis_port_initialized = 1U;
    M3508GroupUpdate(&chassis_motor_group, 0U);
    return 1U;
}

uint8_t ChassisPort_IsReady(void)
{
    uint32_t index;
    uint32_t now_ms;

    if (chassis_port_initialized == 0U)
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        const M3508Feedback_TypeDef *feedback =
            &chassis_motor_group.motor[index].feedback;

        if ((feedback->update_cnt == 0U) ||
            ChassisPort_IsElapsed(
                now_ms,
                feedback->last_update_ms,
                CHASSIS_FEEDBACK_TIMEOUT_MS))
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t ChassisPort_SendMotorRpm(
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT])
{
    uint32_t index;

    if ((chassis_port_initialized == 0U) || (motor_rpm == 0))
    {
        return 0U;
    }

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        M3508GroupSetTarget(
            &chassis_motor_group,
            chassis_motor_id[index],
            motor_rpm[index]);
    }
    M3508GroupUpdate(
        &chassis_motor_group,
        (uint8_t)((chassis_motor_enabled != 0U) &&
                  (CHASSIS_M3508_PID_CONFIG_READY != 0U)));
    return 1U;
}

uint8_t ChassisPort_ReadMotorRpm(
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t *feedback_time_ms)
{
    uint32_t index;
    uint32_t latest_update_ms = 0U;

    if ((motor_rpm == 0) || (feedback_time_ms == 0) ||
        (ChassisPort_IsReady() == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        const M3508Feedback_TypeDef *feedback =
            &chassis_motor_group.motor[index].feedback;
        motor_rpm[index] = (float)feedback->speed_rpm;
        if (feedback->last_update_ms > latest_update_ms)
        {
            latest_update_ms = feedback->last_update_ms;
        }
    }
    *feedback_time_ms = latest_update_ms;
    return 1U;
}

void ChassisPort_SetMotorEnabled(uint8_t enabled)
{
    if (chassis_port_initialized == 0U)
    {
        return;
    }

    chassis_motor_enabled = (uint8_t)(enabled != 0U);
    if (chassis_motor_enabled == 0U)
    {
        M3508GroupUpdate(&chassis_motor_group, 0U);
    }
}
