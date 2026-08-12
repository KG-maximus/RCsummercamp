#include "chassis_task.h"

#include <string.h>

#include "chassis_config.h"
#include "chassis_port.h"
#include "cmsis_os2.h"
#include "main.h"

static osMessageQueueId_t chassis_command_queue;
static osMutexId_t chassis_status_mutex;
static ChassisControl_t chassis_control;
static ChassisTask_Status_t chassis_task_status;
static uint8_t chassis_controller_ready;
static ChassisControl_Result_t chassis_last_command_result;

static uint32_t ChassisTask_MillisecondsToTicks(uint32_t milliseconds)
{
    const uint32_t frequency = osKernelGetTickFreq();
    uint64_t ticks;

    if ((milliseconds == 0U) || (frequency == 0U))
    {
        return 0U;
    }

    ticks = ((uint64_t)milliseconds * (uint64_t)frequency + 999ULL) / 1000ULL;
    return (ticks > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)ticks;
}

static ChassisControl_Config_t ChassisTask_BuildConfig(void)
{
    ChassisControl_Config_t config;

    memset(&config, 0, sizeof(config));
    config.mecanum.wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    config.mecanum.half_wheelbase_m = CHASSIS_HALF_WHEELBASE_M;
    config.mecanum.half_track_m = CHASSIS_HALF_TRACK_M;
    config.mecanum.gear_ratio = CHASSIS_GEAR_RATIO;
    config.mecanum.max_motor_rpm = CHASSIS_MAX_MOTOR_RPM;
    config.mecanum.motor_direction[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] =
        CHASSIS_MOTOR_DIRECTION_FL;
    config.mecanum.motor_direction[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] =
        CHASSIS_MOTOR_DIRECTION_FR;
    config.mecanum.motor_direction[CHASSIS_MECANUM_WHEEL_REAR_LEFT] =
        CHASSIS_MOTOR_DIRECTION_RL;
    config.mecanum.motor_direction[CHASSIS_MECANUM_WHEEL_REAR_RIGHT] =
        CHASSIS_MOTOR_DIRECTION_RR;

    config.max_vx_mps = CHASSIS_MAX_VX_MPS;
    config.max_vy_mps = CHASSIS_MAX_VY_MPS;
    config.max_wz_radps = CHASSIS_MAX_WZ_RADPS;
    config.max_vx_accel_mps2 = CHASSIS_MAX_VX_ACCEL_MPS2;
    config.max_vy_accel_mps2 = CHASSIS_MAX_VY_ACCEL_MPS2;
    config.max_wz_accel_radps2 = CHASSIS_MAX_WZ_ACCEL_RADPS2;

    config.planner_translation.max_acceleration =
        CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2;
    config.planner_translation.max_velocity =
        CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS;
    config.planner_translation.max_jerk =
        CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3;
    config.planner_yaw.max_acceleration =
        CHASSIS_PLAN_YAW_MAX_ACCEL_RADPS2;
    config.planner_yaw.max_velocity = CHASSIS_PLAN_YAW_MAX_SPEED_RADPS;
    config.planner_yaw.max_jerk = CHASSIS_PLAN_YAW_MAX_JERK_RADPS3;

    config.command_timeout_ms = CHASSIS_COMMAND_TIMEOUT_MS;
    config.feedback_timeout_ms = CHASSIS_FEEDBACK_TIMEOUT_MS;
    config.require_motor_feedback = CHASSIS_REQUIRE_MOTOR_FEEDBACK;
    return config;
}

static uint8_t ChassisTask_StateUsesMotor(
    ChassisControl_State_t state)
{
    return (uint8_t)((state == CHASSIS_CONTROL_STATE_IDLE) ||
                     (state == CHASSIS_CONTROL_STATE_VELOCITY) ||
                     (state == CHASSIS_CONTROL_STATE_TRAJECTORY) ||
                     (state == CHASSIS_CONTROL_STATE_STOPPING));
}

static void ChassisTask_PublishStatus(void)
{
    ChassisTask_Status_t snapshot;

    snapshot = chassis_task_status;
    if (chassis_controller_ready != 0U)
    {
        ChassisControl_GetStatus(&chassis_control, &snapshot.control);
    }
    snapshot.last_command_result = chassis_last_command_result;
    snapshot.port_ready = ChassisPort_IsReady();

    if ((chassis_status_mutex != NULL) &&
        (osMutexAcquire(chassis_status_mutex, 0U) == osOK))
    {
        chassis_task_status = snapshot;
        (void)osMutexRelease(chassis_status_mutex);
    }
}

uint8_t ChassisTask_Init(void)
{
    ChassisControl_Config_t config;
    ChassisControl_Result_t result = CHASSIS_CONTROL_NOT_INITIALIZED;

    memset(&chassis_task_status, 0, sizeof(chassis_task_status));
    chassis_task_status.control.state = CHASSIS_CONTROL_STATE_UNINITIALIZED;
    chassis_task_status.control.fault_flags = CHASSIS_FAULT_CONFIG;

    chassis_command_queue = osMessageQueueNew(
        CHASSIS_COMMAND_QUEUE_LENGTH,
        sizeof(ChassisCommand_t),
        NULL);
    chassis_status_mutex = osMutexNew(NULL);
    if ((chassis_command_queue == NULL) || (chassis_status_mutex == NULL))
    {
        return 0U;
    }
    chassis_task_status.rtos_objects_ready = 1U;

    chassis_task_status.port_ready = ChassisPort_Init();
    ChassisPort_SetMotorEnabled(0U);
    if (CHASSIS_APP_CONFIG_READY != 0U)
    {
        config = ChassisTask_BuildConfig();
        result = ChassisControl_Init(&chassis_control, &config, HAL_GetTick());
        if (result == CHASSIS_CONTROL_OK)
        {
            chassis_controller_ready = 1U;
            chassis_task_status.configuration_ready = 1U;
            ChassisControl_GetStatus(
                &chassis_control,
                &chassis_task_status.control);
        }
    }

    chassis_last_command_result = result;
    chassis_task_status.last_command_result = result;
    return 1U;
}

uint8_t ChassisTask_PostCommand(
    const ChassisCommand_t *command,
    uint32_t timeout_ms)
{
    ChassisCommand_t queued_command;
    uint32_t timeout_ticks;
    uint8_t message_priority = 0U;

    if ((command == NULL) || (chassis_command_queue == NULL))
    {
        return 0U;
    }

    queued_command = *command;
    if (queued_command.issued_at_ms == 0U)
    {
        queued_command.issued_at_ms = HAL_GetTick();
    }
    if (queued_command.type == CHASSIS_COMMAND_ESTOP)
    {
        message_priority = 3U;
    }
    else if ((queued_command.type == CHASSIS_COMMAND_DISABLE) ||
             (queued_command.type == CHASSIS_COMMAND_STOP))
    {
        message_priority = 2U;
    }
    else if (queued_command.type == CHASSIS_COMMAND_CLEAR_FAULT)
    {
        message_priority = 1U;
    }
    timeout_ticks = ChassisTask_MillisecondsToTicks(timeout_ms);

    return (uint8_t)(osMessageQueuePut(
        chassis_command_queue,
        &queued_command,
        message_priority,
        timeout_ticks) == osOK);
}

uint8_t ChassisTask_GetStatus(ChassisTask_Status_t *status)
{
    if ((status == NULL) || (chassis_status_mutex == NULL))
    {
        return 0U;
    }

    if (osMutexAcquire(
            chassis_status_mutex,
            ChassisTask_MillisecondsToTicks(10U)) != osOK)
    {
        return 0U;
    }

    *status = chassis_task_status;
    (void)osMutexRelease(chassis_status_mutex);
    return 1U;
}

void ChassisTask_Entry(void *argument)
{
    ChassisCommand_t command;
    ChassisControl_Output_t output;
    ChassisControl_Result_t result;
    float feedback_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    uint32_t feedback_time_ms;
    uint32_t next_wake_tick;
    uint32_t now_ms;
    uint8_t port_ready;

    (void)argument;
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        now_ms = HAL_GetTick();
        port_ready = ChassisPort_IsReady();

        if ((chassis_controller_ready != 0U) &&
            (port_ready != 0U) &&
            ChassisPort_ReadMotorRpm(feedback_rpm, &feedback_time_ms))
        {
            (void)ChassisControl_UpdateMotorFeedback(
                &chassis_control,
                feedback_rpm,
                feedback_time_ms);
        }

        while (osMessageQueueGet(
                   chassis_command_queue,
                   &command,
                   NULL,
                   0U) == osOK)
        {
            if (chassis_controller_ready == 0U)
            {
                chassis_last_command_result =
                    CHASSIS_CONTROL_NOT_INITIALIZED;
                continue;
            }

            if ((command.type == CHASSIS_COMMAND_ENABLE) &&
                (port_ready == 0U))
            {
                ChassisControl_SetExternalFault(
                    &chassis_control,
                    CHASSIS_FAULT_PORT_NOT_READY);
                chassis_last_command_result =
                    CHASSIS_CONTROL_COMMAND_REJECTED;
                continue;
            }

            chassis_last_command_result =
                ChassisControl_SubmitCommand(
                    &chassis_control,
                    &command,
                    now_ms);
        }

        if (chassis_controller_ready != 0U)
        {
            if ((port_ready == 0U) &&
                ChassisTask_StateUsesMotor(chassis_control.status.state))
            {
                ChassisControl_SetExternalFault(
                    &chassis_control,
                    CHASSIS_FAULT_PORT_NOT_READY);
            }

            result = ChassisControl_Step(
                &chassis_control,
                now_ms,
                (float)CHASSIS_CONTROL_PERIOD_MS * 0.001f,
                &output);
            if ((result == CHASSIS_CONTROL_OK) &&
                (output.send_motor_targets != 0U) &&
                (port_ready != 0U))
            {
                if (!ChassisPort_SendMotorRpm(output.motor_rpm))
                {
                    const float zero_rpm[CHASSIS_MECANUM_WHEEL_COUNT] =
                        {0.0f, 0.0f, 0.0f, 0.0f};
                    ChassisControl_SetExternalFault(
                        &chassis_control,
                        CHASSIS_FAULT_MOTOR_TX);
                    (void)ChassisPort_SendMotorRpm(zero_rpm);
                }
            }

            ChassisPort_SetMotorEnabled((uint8_t)(
                (port_ready != 0U) &&
                ChassisTask_StateUsesMotor(chassis_control.status.state)));
        }
        else
        {
            ChassisPort_SetMotorEnabled(0U);
        }

        ChassisTask_PublishStatus();
        next_wake_tick += ChassisTask_MillisecondsToTicks(
            CHASSIS_CONTROL_PERIOD_MS);
        (void)osDelayUntil(next_wake_tick);
    }
}
