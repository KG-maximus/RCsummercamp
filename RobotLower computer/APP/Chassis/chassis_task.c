#include "chassis_task.h"

#include <string.h>

#include "chassis_config.h"
#include "chassis_port.h"
#include "cmsis_os2.h"
#include "main.h"

/*
 * ============================ 底盘任务职责 ============================
 *
 * 这是 APP/Chassis 的实时调度层。它不在 FDCAN 回调中做控制，而是在固定
 * 周期线程中执行以下顺序：
 *   1. 读取 Port 已缓存的电机反馈；
 *   2. 取尽命令消息队列，交由 ChassisControl 做执行前安全检查；
 *   3. 调用 ChassisControl_Step 计算四路目标 rpm；
 *   4. 通过 ChassisPort 发给 M3508/C620；
 *   5. 发布带 Mutex 保护的状态快照；
 *   6. osDelayUntil 保持稳定的 CHASSIS_CONTROL_PERIOD_MS 周期。
 *
 * 不要在本循环加入 HAL_Delay、长时间 printf 或阻塞式外设收发，否则会造成
 * 控制周期抖动、命令超时或 S 曲线积分异常。
 * ======================================================================
 */
static osMessageQueueId_t chassis_command_queue;  /* 跨任务提交 ChassisCommand_t 的 CMSIS 队列。 */
static osMutexId_t chassis_status_mutex;          /* 保护 chassis_task_status 快照的互斥锁。 */
static ChassisControl_t chassis_control;          /* 仅由控制任务写入的控制器工作对象。 */
static ChassisTask_Status_t chassis_task_status;  /* 供外部读取的状态快照缓存。 */
static uint8_t chassis_controller_ready;          /* chassis_config 有效且 ChassisControl_Init 成功。 */
static ChassisControl_Result_t chassis_last_command_result; /* 最近处理命令的结果。 */

static uint32_t ChassisTask_MillisecondsToTicks(uint32_t milliseconds)
{
    const uint32_t frequency = osKernelGetTickFreq();
    uint64_t ticks;

    if ((milliseconds == 0U) || (frequency == 0U))
    {
        return 0U;
    }

    /* 向上取整，确保非零毫秒不会因为 Tick 分辨率而被错误换算成 0 Tick。 */
    ticks = ((uint64_t)milliseconds * (uint64_t)frequency + 999ULL) / 1000ULL;
    return (ticks > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)ticks;
}

static ChassisControl_Config_t ChassisTask_BuildConfig(void)
{
    ChassisControl_Config_t config;

    /*
     * 只在 APP_CONFIG_READY 打开后调用。这里刻意逐字段复制，以便把
     * chassis_config.h 作为唯一可审查的真车参数入口，避免其他模块暗改值。
     */
    memset(&config, 0, sizeof(config));
    config.mecanum.wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    config.mecanum.half_wheelbase_m = CHASSIS_HALF_WHEELBASE_M;
    config.mecanum.half_track_m = CHASSIS_HALF_TRACK_M;
    config.mecanum.gear_ratio = CHASSIS_GEAR_RATIO;
    config.mecanum.max_motor_rpm = CHASSIS_MAX_MOTOR_RPM;
    /* 再次明确运动学数组顺序：FL、FR、RL、RR。 */
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
    /* 这些状态需要保持 Port 输出“已允许”；DISABLED/FAULT 必须禁用 PID 输出。 */
    return (uint8_t)((state == CHASSIS_CONTROL_STATE_IDLE) ||
                     (state == CHASSIS_CONTROL_STATE_VELOCITY) ||
                     (state == CHASSIS_CONTROL_STATE_TRAJECTORY) ||
                     (state == CHASSIS_CONTROL_STATE_STOPPING));
}

static void ChassisTask_PublishStatus(void)
{
    ChassisTask_Status_t snapshot;

    /*
     * 先在局部变量中构造完整快照，再短时间持锁替换，缩短读取任务被阻塞的
     * 时间。控制器本身只由本任务写，所以不需要给它另加一把 Mutex。
     */
    snapshot = chassis_task_status;
    if (chassis_controller_ready != 0U)
    {
        ChassisControl_GetStatus(&chassis_control, &snapshot.control);
    }
    snapshot.last_command_result = chassis_last_command_result;
    /* port_ready 每周期重新检查，便于上位机区分“配置错误”与“CAN 反馈掉线”。 */
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

    /*
     * 初始化阶段只创建 RTOS 对象和驱动/控制器，不启动电机。线程应由
     * freertos.c 在 osKernelStart 后创建并执行 ChassisTask_Entry。
     */
    memset(&chassis_task_status, 0, sizeof(chassis_task_status));
    chassis_task_status.control.state = CHASSIS_CONTROL_STATE_UNINITIALIZED;
    chassis_task_status.control.fault_flags = CHASSIS_FAULT_CONFIG;

    /* 队列的元素是完整 ChassisCommand_t，优先级由 PostCommand 填写。 */
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

    /*
     * 端口即使初始化成功，也可能尚未收到电调反馈；先强制禁止电机输出，
     * 防止上一轮测试残留的目标在系统启动时生效。
     */
    chassis_task_status.port_ready = ChassisPort_Init();
    ChassisPort_SetMotorEnabled(0U);
    /* 配置总开关为 0 时故意跳过控制器初始化，保留“只观察 CAN 反馈”的安全模式。 */
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

    /*
     * 此接口只负责入队；命令来源仲裁、序号去重和业务有效期由上位机/总
     * 状态机完成。控制任务随后只检查命令格式、当前状态与硬件安全条件。
     */
    if ((command == NULL) || (chassis_command_queue == NULL))
    {
        return 0U;
    }

    queued_command = *command;
    /* 调用者未填时间戳时，在入队时补本机 HAL tick，便于超时校验。 */
    if (queued_command.issued_at_ms == 0U)
    {
        queued_command.issued_at_ms = HAL_GetTick();
    }
    /* CMSIS 队列优先级数值越大越先被取出，安全类命令应优先到达控制器。 */
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
    /*
     * 读取任务最多等待约 10 ms 获取 Mutex。不能为“拿到最新状态”而无限
     * 阻塞上位机或日志任务；失败时由调用者跳过本轮即可。
     */
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

    /* argument 当前未使用；next_wake_tick 是 osDelayUntil 的绝对唤醒基准。 */
    (void)argument;
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        /* 周期开始时先检查 Port 在线状态，再读取同一时刻附近的反馈快照。 */
        now_ms = HAL_GetTick();
        port_ready = ChassisPort_IsReady();

        /*
         * 反馈帧已经由 FDCAN 回调写入 M3508 缓存；这里在任务上下文中完成
         * 正运动学和控制器状态更新，避免中断中进行浮点控制运算。
         */
        if ((chassis_controller_ready != 0U) &&
            (port_ready != 0U) &&
            ChassisPort_ReadMotorRpm(feedback_rpm, &feedback_time_ms))
        {
            (void)ChassisControl_UpdateMotorFeedback(
                &chassis_control,
                feedback_rpm,
                feedback_time_ms);
        }

        /* 一次取尽当前队列，保证高优先级急停不因旧速度命令滞留到下一周期。 */
        while (osMessageQueueGet(
                   chassis_command_queue,
                   &command,
                   NULL,
                   0U) == osOK)
        {
            /* 配置未就绪时只报告错误，绝不尝试解释或执行运动命令。 */
            if (chassis_controller_ready == 0U)
            {
                chassis_last_command_result =
                    CHASSIS_CONTROL_NOT_INITIALIZED;
                continue;
            }

            /* ENABLE 需要四轮反馈在线；其他命令的细节仲裁交给控制器。 */
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

        /* 控制器就绪后每周期必做一次 Step；未就绪则持续禁止 Port 输出。 */
        if (chassis_controller_ready != 0U)
        {
            /* 运行期间 Port 掉线立即报告外部故障，避免继续按旧反馈控制。 */
            if ((port_ready == 0U) &&
                ChassisTask_StateUsesMotor(chassis_control.status.state))
            {
                ChassisControl_SetExternalFault(
                    &chassis_control,
                    CHASSIS_FAULT_PORT_NOT_READY);
            }

            /* delta_time 使用配置周期换算成秒，单位必须与运动学/加速度参数一致。 */
            result = ChassisControl_Step(
                &chassis_control,
                now_ms,
                (float)CHASSIS_CONTROL_PERIOD_MS * 0.001f,
                &output);
            if ((result == CHASSIS_CONTROL_OK) &&
                (output.send_motor_targets != 0U) &&
                (port_ready != 0U))
            {
                /* 发送接口失败时立即锁存 TX 故障并再尝试一帧零目标。 */
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

            /* 状态机只在运行/停车态允许 PID 输出；FAULT/DISABLED 会立刻清零电流。 */
            ChassisPort_SetMotorEnabled((uint8_t)(
                (port_ready != 0U) &&
                ChassisTask_StateUsesMotor(chassis_control.status.state)));
        }
        else
        {
            ChassisPort_SetMotorEnabled(0U);
        }

        /* 发布快照后采用绝对时间延时，减小本周期计算时间造成的累计漂移。 */
        ChassisTask_PublishStatus();
        next_wake_tick += ChassisTask_MillisecondsToTicks(
            CHASSIS_CONTROL_PERIOD_MS);
        (void)osDelayUntil(next_wake_tick);
    }
}
