#include "chassis.h"

#include <string.h>

#include "bsp_callback.h"
#include "chassis_config.h"
#include "cmsis_os2.h"
#include "main.h"

/*
 * =========================== 文件内部逻辑顺序 ===========================
 *
 * 为减少原来 control/port/task 三套文件来回跳转，本文件按真实数据流排列：
 *   1. 模块私有对象和轮子-ID 映射；
 *   2. M3508/FDCAN 底层适配；
 *   3. chassis_config 到控制器配置的转换；
 *   4. 状态快照发布；
 *   5. 对上位机/总状态机公开的命令接口；
 *   6. 固定周期 FreeRTOS 任务。
 *
 * 纯麦轮数学仍在 chassis_mecanum.c，纯控制状态机仍在 chassis_control.c。
 * 两者均不接触 FDCAN 和 RTOS，方便分别阅读和测试。
 * ========================================================================
 */

ChassisState_TypeDef ChassisState;

static osMessageQueueId_t chassis_command_queue; /* 上层到控制任务的命令队列。 */
static osMutexId_t chassis_state_mutex;           /* 保护 ChassisState 一致快照。 */
static ChassisControl_t chassis_control;          /* 仅由 ChassisTask 写的控制器。 */
static M3508Group_TypeDef chassis_motor_group;    /* 四台 C620/M3508 及反馈缓存。 */
/*
 * 原始 M3508 驱动只记录反馈累计次数，不记录接收时刻。反馈超时属于底盘的
 * 安全策略，因此在适配层按 FL/FR/RL/RR 单独保存最近一帧的 HAL tick。
 */
static volatile uint32_t
    chassis_feedback_update_ms[CHASSIS_MECANUM_WHEEL_COUNT];
static ChassisControl_Result_t chassis_last_command_result;
static uint32_t chassis_command_sequence;         /* 便捷接口自动生成的本地序号。 */
static uint8_t chassis_initialized;
static uint8_t chassis_controller_ready;
static uint8_t chassis_port_initialized;
static uint8_t chassis_motor_enabled;

static const uint8_t chassis_motor_id[CHASSIS_MECANUM_WHEEL_COUNT] =
{
    CHASSIS_M3508_ID_FL,
    CHASSIS_M3508_ID_FR,
    CHASSIS_M3508_ID_RL,
    CHASSIS_M3508_ID_RR
};

/* ======================== 1. 通用私有辅助函数 ======================== */

static uint32_t Chassis_MillisecondsToTicks(uint32_t milliseconds)
{
    const uint32_t frequency = osKernelGetTickFreq();
    uint64_t ticks;

    if ((milliseconds == 0U) || (frequency == 0U))
    {
        return 0U;
    }

    /* 向上取整，防止非零毫秒在低 Tick 频率下被换算成 0。 */
    ticks = ((uint64_t)milliseconds * (uint64_t)frequency + 999ULL) / 1000ULL;
    return (ticks > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)ticks;
}

static uint8_t Chassis_HasElapsed(
    uint32_t now_ms,
    uint32_t timestamp_ms,
    uint32_t timeout_ms)
{
    /* 无符号减法可自然跨越 HAL tick 回卷；时间戳为 0 表示从未收到反馈。 */
    return (uint8_t)((timestamp_ms == 0U) ||
                     ((uint32_t)(now_ms - timestamp_ms) > timeout_ms));
}

static uint8_t Chassis_StateUsesMotor(ChassisControl_State_t state)
{
    return (uint8_t)((state == CHASSIS_CONTROL_STATE_IDLE) ||
                     (state == CHASSIS_CONTROL_STATE_VELOCITY) ||
                     (state == CHASSIS_CONTROL_STATE_TRAJECTORY) ||
                     (state == CHASSIS_CONTROL_STATE_STOPPING));
}

static uint8_t Chassis_MotorGroupBaseId(void)
{
    return (CHASSIS_M3508_CONTROL_ID == M3508_CTRL_ID_1TO4) ? 1U : 5U;
}

static M3508_TypeDef *Chassis_GetMotorByWheel(uint32_t wheel_index)
{
    const uint8_t base_id = Chassis_MotorGroupBaseId();
    const uint8_t id = chassis_motor_id[wheel_index];

    if ((id < base_id) || (id >= (uint8_t)(base_id + M3508_GROUP_SIZE)))
    {
        return 0;
    }
    return &chassis_motor_group.motor[id - base_id];
}

static uint8_t Chassis_CopyMotorFeedback(
    uint32_t wheel_index,
    M3508Feedback_TypeDef *feedback,
    uint32_t *feedback_time_ms)
{
    M3508_TypeDef *motor;
    uint32_t primask;

    if ((wheel_index >= CHASSIS_MECANUM_WHEEL_COUNT) || (feedback == 0))
    {
        return 0U;
    }
    motor = Chassis_GetMotorByWheel(wheel_index);
    if (motor == 0)
    {
        return 0U;
    }

    /*
     * FDCAN 回调可能随时更新 feedback。这里只屏蔽中断复制一个很小的结构，
     * 防止任务读到角度来自新帧、转速却仍来自旧帧的混合状态。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor->feedback;
    if (feedback_time_ms != 0)
    {
        *feedback_time_ms = chassis_feedback_update_ms[wheel_index];
    }
    __set_PRIMASK(primask);
    return 1U;
}

/* ======================== 2. M3508/FDCAN 适配 ======================== */

static uint8_t Chassis_FeedbackIdToWheel(
    uint32_t std_id,
    uint32_t *wheel_index)
{
    uint32_t wheel;

    if (wheel_index == 0)
    {
        return 0U;
    }

    /*
     * C620 的反馈标准 ID = 0x200 + 电调 ID。这里按 chassis_config.h 中
     * 的物理轮映射查找，不能直接用 id-1 当作轮序，否则交换电调 ID 后
     * FL/FR/RL/RR 的反馈会错位。
     */
    for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
    {
        if (std_id ==
            (uint32_t)(M3508_FEEDBACK_ID_BASE + chassis_motor_id[wheel]))
        {
            *wheel_index = wheel;
            return 1U;
        }
    }
    return 0U;
}

static void Chassis_ResetPIDRuntime(PID_TypeDef *pid)
{
    if (pid == 0)
    {
        return;
    }

    /*
     * 只清除 PID 的运行历史，保留参考驱动已经验证过的 kp/ki/kd 和限幅。
     * 这样急停或反馈超时后再次使能，不会继承停机前的积分和微分历史。
     */
    pid->tar = 0.0f;
    pid->act = 0.0f;
    pid->error0 = 0.0f;
    pid->error1 = 0.0f;
    pid->error_i = 0.0f;
    pid->out = 0.0f;
}

static void Chassis_ResetMotorControlRuntime(void)
{
    uint32_t motor_index;

    for (motor_index = 0U; motor_index < M3508_GROUP_SIZE; ++motor_index)
    {
        M3508Control_TypeDef *control =
            &chassis_motor_group.motor[motor_index].control;

        control->speed_target = 0.0f;
        control->current_output = 0;
        Chassis_ResetPIDRuntime(&control->pid.outer);
        Chassis_ResetPIDRuntime(&control->pid.inner);
    }
}

static void Chassis_PortSendZeroCurrent(void)
{
    static const uint8_t zero_current_data[8] =
        {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    /*
     * 原始 M3508GroupUpdate() 会无条件执行四台电机的级联 PID，不能用它
     * 表达“禁用”。底盘禁用、故障或 READY 未确认时，直接发送 DJI 协议
     * 的四路零电流组帧，并清空控制器运行历史。
     */
    Chassis_ResetMotorControlRuntime();
    FDCANSendStandard(
        CHASSIS_M3508_CAN_HANDLE,
        CHASSIS_M3508_CONTROL_ID,
        zero_current_data,
        8U);
}

static void Chassis_FDCANRxHandler(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint32_t std_id,
    const uint8_t data[8])
{
    uint32_t wheel_index;

    /*
     * 该函数在 HAL FDCAN 接收回调上下文运行，只过滤总线并解析一帧反馈。
     * 不在这里做 PID、运动学、队列等待或 CAN 发送，避免中断执行时间失控。
     */
    if (fdcan_handle != CHASSIS_M3508_CAN_HANDLE)
    {
        return;
    }

    if (M3508GroupParseFeedback(
            &chassis_motor_group,
            std_id,
            data) == 0U)
    {
        return;
    }

    if (Chassis_FeedbackIdToWheel(std_id, &wheel_index) != 0U)
    {
        chassis_feedback_update_ms[wheel_index] = HAL_GetTick();
    }
}

static uint8_t Chassis_IsMotorIdConfigValid(void)
{
    uint32_t first;
    uint32_t second;
    const uint8_t base_id = Chassis_MotorGroupBaseId();

    if ((CHASSIS_M3508_CONTROL_ID != M3508_CTRL_ID_1TO4) &&
        (CHASSIS_M3508_CONTROL_ID != M3508_CTRL_ID_5TO8))
    {
        return 0U;
    }

    for (first = 0U; first < CHASSIS_MECANUM_WHEEL_COUNT; ++first)
    {
        if ((chassis_motor_id[first] < base_id) ||
            (chassis_motor_id[first] >= (uint8_t)(base_id + M3508_GROUP_SIZE)))
        {
            return 0U;
        }
        for (second = first + 1U; second < CHASSIS_MECANUM_WHEEL_COUNT; ++second)
        {
            if (chassis_motor_id[first] == chassis_motor_id[second])
            {
                return 0U;
            }
        }
    }
    return 1U;
}

static uint8_t Chassis_PortInit(void)
{
    const uint8_t base_id = Chassis_MotorGroupBaseId();

    if (chassis_port_initialized != 0U)
    {
        return 1U;
    }
    if (Chassis_IsMotorIdConfigValid() == 0U)
    {
        return 0U;
    }

    /* 完整采用参考 M3508 驱动内部定义的默认级联 PID，不在底盘层重复配置。 */
    M3508GroupInit(
        &chassis_motor_group,
        CHASSIS_M3508_CAN_HANDLE,
        CHASSIS_M3508_CONTROL_ID);
    if (BSPCallback_RegisterFDCANRxHandler(Chassis_FDCANRxHandler) == 0U)
    {
        return 0U;
    }

    /* 同组四个反馈 ID 是连续的：1~4 对应 0x201~0x204，5~8 同理。 */
    FDCANStandardInit(
        CHASSIS_M3508_CAN_HANDLE,
        M3508_FEEDBACK_ID_BASE + base_id,
        M3508_FEEDBACK_ID_BASE + base_id + M3508_GROUP_SIZE - 1U);

    chassis_motor_enabled = 0U;
    chassis_port_initialized = 1U;
    Chassis_PortSendZeroCurrent(); /* 上电先明确发送四路零电流。 */
    return 1U;
}

static uint8_t Chassis_PortIsReady(void)
{
    uint32_t wheel;
    const uint32_t now_ms = HAL_GetTick();

    if (chassis_port_initialized == 0U)
    {
        return 0U;
    }

    for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
    {
        M3508Feedback_TypeDef feedback;
        uint32_t feedback_time_ms;
        if ((Chassis_CopyMotorFeedback(
                 wheel,
                 &feedback,
                 &feedback_time_ms) == 0U) ||
            (feedback.update_cnt == 0U) ||
            Chassis_HasElapsed(
                now_ms,
                feedback_time_ms,
                CHASSIS_FEEDBACK_TIMEOUT_MS))
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t Chassis_PortReadFeedback(
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t *feedback_time_ms)
{
    uint32_t wheel;
    uint32_t newest_ms = 0U;

    if ((motor_rpm == 0) || (feedback_time_ms == 0) ||
        (Chassis_PortIsReady() == 0U))
    {
        return 0U;
    }

    for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
    {
        M3508Feedback_TypeDef feedback;
        uint32_t wheel_feedback_ms;
        if (Chassis_CopyMotorFeedback(
                wheel,
                &feedback,
                &wheel_feedback_ms) == 0U)
        {
            return 0U;
        }
        motor_rpm[wheel] = (float)feedback.speed_rpm;
        if (wheel_feedback_ms > newest_ms)
        {
            newest_ms = wheel_feedback_ms;
        }
    }
    *feedback_time_ms = newest_ms;
    return 1U;
}

static uint8_t Chassis_PortSendMotorRpm(
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT])
{
    uint32_t wheel;

    if ((chassis_port_initialized == 0U) || (motor_rpm == 0))
    {
        return 0U;
    }

    for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
    {
        M3508GroupSetTarget(
            &chassis_motor_group,
            chassis_motor_id[wheel],
            motor_rpm[wheel]);
    }

    /*
     * READY 默认是 0U。只有状态机允许输出且 PID 已人工确认，才调用原始
     * 驱动执行级联 PID；其余情况由适配层发送确定的全零电流帧。
     */
    if ((chassis_motor_enabled != 0U) &&
        (CHASSIS_M3508_PID_CONFIG_READY != 0U))
    {
        M3508GroupUpdate(&chassis_motor_group);
    }
    else
    {
        Chassis_PortSendZeroCurrent();
    }
    return 1U;
}

static void Chassis_PortSetMotorEnabled(uint8_t enabled)
{
    if (chassis_port_initialized == 0U)
    {
        return;
    }

    chassis_motor_enabled = (uint8_t)(enabled != 0U);
    if (chassis_motor_enabled == 0U)
    {
        /* 禁用动作立即发零电流，不能等待下一个 3 ms 控制周期。 */
        Chassis_PortSendZeroCurrent();
    }
}

/* ======================== 3. 构建控制器配置 ======================== */

static ChassisControl_Config_t Chassis_BuildControlConfig(void)
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

/* ======================== 4. 发布完整状态快照 ======================== */

static void Chassis_PublishState(void)
{
    ChassisState_TypeDef snapshot = ChassisState;
    ChassisControl_Status_t control_status;
    uint32_t newest_feedback_ms = 0U;
    uint32_t wheel;

    snapshot.initialized = chassis_initialized;
    snapshot.configuration_ready = chassis_controller_ready;
    snapshot.rtos_objects_ready = (uint8_t)(
        (chassis_command_queue != 0) && (chassis_state_mutex != 0));
    snapshot.port_ready = Chassis_PortIsReady();
    snapshot.motor_output_enabled = (uint8_t)(
        (chassis_motor_enabled != 0U) &&
        (CHASSIS_M3508_PID_CONFIG_READY != 0U));
    snapshot.last_command_result = chassis_last_command_result;

    if (chassis_controller_ready != 0U)
    {
        ChassisControl_GetStatus(&chassis_control, &control_status);
        snapshot.state = control_status.state;
        snapshot.fault_flags = control_status.fault_flags;
        snapshot.active_source = control_status.active_source;
        snapshot.last_sequence = control_status.last_sequence;
        snapshot.last_command_ms = control_status.last_command_ms;
        snapshot.last_feedback_ms = control_status.last_feedback_ms;
        snapshot.pose = control_status.pose;
        snapshot.target_pose = control_status.target_pose;
        snapshot.target_velocity = control_status.requested_body_velocity;
        snapshot.command_velocity = control_status.commanded_body_velocity;
        snapshot.feedback_velocity = control_status.actual_body_velocity;
        snapshot.motor_scale = control_status.motor_scale;
        snapshot.feedback_valid = control_status.feedback_valid;
        memcpy(
            snapshot.target_motor_rpm,
            control_status.target_motor_rpm,
            sizeof(snapshot.target_motor_rpm));
        memcpy(
            snapshot.feedback_motor_rpm,
            control_status.feedback_motor_rpm,
            sizeof(snapshot.feedback_motor_rpm));
    }

    for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
    {
        M3508Feedback_TypeDef feedback;
        uint32_t wheel_feedback_ms;
        if (Chassis_CopyMotorFeedback(
                wheel,
                &feedback,
                &wheel_feedback_ms) != 0U)
        {
            snapshot.motor_feedback[wheel] = feedback;
            snapshot.motor_feedback_ms[wheel] = wheel_feedback_ms;
            /*
             * 即使机械参数尚未填写、控制器未初始化，也把 CAN 原始轮速直接
             * 发布出来。这使 READY=0 的安全联调模式仍能确认四个 ID 和方向。
             */
            snapshot.feedback_motor_rpm[wheel] =
                (float)feedback.speed_rpm;
            if (wheel_feedback_ms > newest_feedback_ms)
            {
                newest_feedback_ms = wheel_feedback_ms;
            }
        }
    }
    if (chassis_controller_ready == 0U)
    {
        snapshot.last_feedback_ms = newest_feedback_ms;
        snapshot.feedback_valid = snapshot.port_ready;
    }

    if ((chassis_state_mutex != 0) &&
        (osMutexAcquire(chassis_state_mutex, 0U) == osOK))
    {
        snapshot.update_count = ChassisState.update_count + 1U;
        ChassisState = snapshot;
        (void)osMutexRelease(chassis_state_mutex);
    }
}

/* ======================== 5. 对上公共接口 ======================== */

uint8_t ChassisInit(void)
{
    ChassisControl_Config_t config;
    ChassisControl_Result_t result = CHASSIS_CONTROL_NOT_INITIALIZED;

    if (chassis_initialized != 0U)
    {
        return 1U;
    }

    memset(&ChassisState, 0, sizeof(ChassisState));
    ChassisState.state = CHASSIS_CONTROL_STATE_UNINITIALIZED;
    ChassisState.fault_flags = CHASSIS_FAULT_CONFIG;
    ChassisState.motor_scale = 1.0f;

    chassis_command_queue = osMessageQueueNew(
        CHASSIS_COMMAND_QUEUE_LENGTH,
        sizeof(ChassisCommand_t),
        0);
    chassis_state_mutex = osMutexNew(0);
    if ((chassis_command_queue == 0) || (chassis_state_mutex == 0))
    {
        return 0U;
    }
    ChassisState.rtos_objects_ready = 1U;

    if (Chassis_PortInit() == 0U)
    {
        ChassisState.fault_flags |= CHASSIS_FAULT_PORT_NOT_READY;
        return 0U;
    }
    Chassis_PortSetMotorEnabled(0U);

    /* 总开关未确认时不初始化运动控制器，但仍允许接收并观察 M3508 反馈。 */
    if (CHASSIS_APP_CONFIG_READY != 0U)
    {
        config = Chassis_BuildControlConfig();
        result = ChassisControl_Init(&chassis_control, &config, HAL_GetTick());
        if (result == CHASSIS_CONTROL_OK)
        {
            chassis_controller_ready = 1U;
            ChassisState.configuration_ready = 1U;
        }
    }

    chassis_last_command_result = result;
    ChassisState.last_command_result = result;
    chassis_initialized = 1U;
    ChassisState.initialized = 1U;
    return 1U;
}

uint8_t ChassisPostCommand(
    const ChassisCommand_t *command,
    uint32_t timeout_ms)
{
    ChassisCommand_t queued_command;
    uint8_t priority = 0U;

    if ((command == 0) || (chassis_command_queue == 0))
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
        priority = 3U;
    }
    else if ((queued_command.type == CHASSIS_COMMAND_DISABLE) ||
             (queued_command.type == CHASSIS_COMMAND_STOP))
    {
        priority = 2U;
    }
    else if (queued_command.type == CHASSIS_COMMAND_CLEAR_FAULT)
    {
        priority = 1U;
    }

    return (uint8_t)(osMessageQueuePut(
        chassis_command_queue,
        &queued_command,
        priority,
        Chassis_MillisecondsToTicks(timeout_ms)) == osOK);
}

static uint8_t Chassis_PostSimpleCommand(
    ChassisCommandType_t type,
    ChassisCommandSource_t source,
    uint32_t timeout_ms)
{
    ChassisCommand_t command;
    memset(&command, 0, sizeof(command));
    command.sequence = ++chassis_command_sequence;
    command.source = source;
    command.type = type;
    command.frame = CHASSIS_FRAME_BODY;
    return ChassisPostCommand(&command, timeout_ms);
}

uint8_t ChassisEnable(ChassisCommandSource_t source, uint32_t timeout_ms)
{
    return Chassis_PostSimpleCommand(CHASSIS_COMMAND_ENABLE, source, timeout_ms);
}

uint8_t ChassisDisable(ChassisCommandSource_t source, uint32_t timeout_ms)
{
    return Chassis_PostSimpleCommand(CHASSIS_COMMAND_DISABLE, source, timeout_ms);
}

uint8_t ChassisStop(ChassisCommandSource_t source, uint32_t timeout_ms)
{
    return Chassis_PostSimpleCommand(CHASSIS_COMMAND_STOP, source, timeout_ms);
}

uint8_t ChassisEmergencyStop(uint32_t timeout_ms)
{
    return Chassis_PostSimpleCommand(
        CHASSIS_COMMAND_ESTOP,
        CHASSIS_SOURCE_SAFETY,
        timeout_ms);
}

uint8_t ChassisClearFault(ChassisCommandSource_t source, uint32_t timeout_ms)
{
    return Chassis_PostSimpleCommand(CHASSIS_COMMAND_CLEAR_FAULT, source, timeout_ms);
}

uint8_t ChassisSetBodyVelocity(
    float vx_mps,
    float vy_mps,
    float wz_radps,
    ChassisCommandSource_t source,
    uint32_t timeout_ms)
{
    ChassisCommand_t command;
    memset(&command, 0, sizeof(command));
    command.sequence = ++chassis_command_sequence;
    command.source = source;
    command.type = CHASSIS_COMMAND_BODY_VELOCITY;
    command.frame = CHASSIS_FRAME_BODY;
    command.payload.body_velocity.vx_mps = vx_mps;
    command.payload.body_velocity.vy_mps = vy_mps;
    command.payload.body_velocity.wz_radps = wz_radps;
    return ChassisPostCommand(&command, timeout_ms);
}

uint8_t ChassisSetRelativePose(
    float x_m,
    float y_m,
    float yaw_rad,
    ChassisReferenceFrame_t frame,
    ChassisCommandSource_t source,
    uint32_t timeout_ms)
{
    ChassisCommand_t command;
    memset(&command, 0, sizeof(command));
    command.sequence = ++chassis_command_sequence;
    command.source = source;
    command.type = CHASSIS_COMMAND_MOVE_RELATIVE;
    command.frame = frame;
    command.payload.pose.x_m = x_m;
    command.payload.pose.y_m = y_m;
    command.payload.pose.yaw_rad = yaw_rad;
    return ChassisPostCommand(&command, timeout_ms);
}

uint8_t ChassisSetAbsolutePose(
    float x_m,
    float y_m,
    float yaw_rad,
    ChassisCommandSource_t source,
    uint32_t timeout_ms)
{
    ChassisCommand_t command;
    memset(&command, 0, sizeof(command));
    command.sequence = ++chassis_command_sequence;
    command.source = source;
    command.type = CHASSIS_COMMAND_MOVE_ABSOLUTE;
    command.frame = CHASSIS_FRAME_ODOM;
    command.payload.pose.x_m = x_m;
    command.payload.pose.y_m = y_m;
    command.payload.pose.yaw_rad = yaw_rad;
    return ChassisPostCommand(&command, timeout_ms);
}

uint8_t ChassisGetState(ChassisState_TypeDef *state)
{
    if ((state == 0) || (chassis_state_mutex == 0))
    {
        return 0U;
    }
    if (osMutexAcquire(
            chassis_state_mutex,
            Chassis_MillisecondsToTicks(10U)) != osOK)
    {
        return 0U;
    }
    *state = ChassisState;
    (void)osMutexRelease(chassis_state_mutex);
    return 1U;
}

/* ======================== 6. FreeRTOS 固定周期任务 ======================== */

void ChassisTask(void *argument)
{
    ChassisCommand_t command;
    ChassisControl_Output_t output;
    ChassisControl_Result_t result;
    float feedback_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    uint32_t feedback_time_ms;
    uint32_t next_wake_tick = osKernelGetTickCount();
    uint32_t now_ms;
    uint8_t port_ready;
    uint8_t motor_should_enable;

    (void)argument;

    for (;;)
    {
        now_ms = HAL_GetTick();
        port_ready = Chassis_PortIsReady();

        /* FDCAN 回调只更新缓存；正运动学和状态更新在普通任务上下文执行。 */
        if ((chassis_controller_ready != 0U) &&
            (port_ready != 0U) &&
            Chassis_PortReadFeedback(feedback_rpm, &feedback_time_ms))
        {
            (void)ChassisControl_UpdateMotorFeedback(
                &chassis_control,
                feedback_rpm,
                feedback_time_ms);
        }

        /* 取尽本周期已有命令，使急停不被较早的普通速度命令阻塞。 */
        while (osMessageQueueGet(
                   chassis_command_queue,
                   &command,
                   0,
                   0U) == osOK)
        {
            if (chassis_controller_ready == 0U)
            {
                chassis_last_command_result = CHASSIS_CONTROL_NOT_INITIALIZED;
                continue;
            }
            if ((command.type == CHASSIS_COMMAND_ENABLE) && (port_ready == 0U))
            {
                ChassisControl_SetExternalFault(
                    &chassis_control,
                    CHASSIS_FAULT_PORT_NOT_READY);
                chassis_last_command_result = CHASSIS_CONTROL_COMMAND_REJECTED;
                continue;
            }
            chassis_last_command_result = ChassisControl_SubmitCommand(
                &chassis_control,
                &command,
                now_ms);
        }

        if (chassis_controller_ready != 0U)
        {
            if ((port_ready == 0U) &&
                Chassis_StateUsesMotor(chassis_control.status.state))
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

            /*
             * 必须先根据“本周期”的状态决定是否允许输出，再调用发送函数。
             * 这样反馈刚超时、急停或状态机刚进入 FAULT 的同一周期就会发
             * 零电流，不会因为沿用上一周期的使能标志而多执行一次 PID。
             */
            motor_should_enable = (uint8_t)(
                (result == CHASSIS_CONTROL_OK) &&
                (port_ready != 0U) &&
                Chassis_StateUsesMotor(chassis_control.status.state));
            Chassis_PortSetMotorEnabled(motor_should_enable);

            if ((result == CHASSIS_CONTROL_OK) &&
                (output.send_motor_targets != 0U) &&
                (motor_should_enable != 0U))
            {
                if (Chassis_PortSendMotorRpm(output.motor_rpm) == 0U)
                {
                    const float zero_rpm[CHASSIS_MECANUM_WHEEL_COUNT] =
                        {0.0f, 0.0f, 0.0f, 0.0f};
                    ChassisControl_SetExternalFault(
                        &chassis_control,
                        CHASSIS_FAULT_MOTOR_TX);
                    (void)Chassis_PortSendMotorRpm(zero_rpm);
                }
            }
        }
        else
        {
            Chassis_PortSetMotorEnabled(0U);
        }

        Chassis_PublishState();
        next_wake_tick += Chassis_MillisecondsToTicks(CHASSIS_CONTROL_PERIOD_MS);
        (void)osDelayUntil(next_wake_tick);
    }
}
