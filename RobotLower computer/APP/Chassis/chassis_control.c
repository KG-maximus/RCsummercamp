#include "chassis_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * ========================== 底盘控制器主流程 ==========================
 *
 * 本文件不直接读写 FDCAN，也不直接计算 C620 电流。它处在任务层和运动学层
 * 之间，职责是把“语义命令”变成安全的四轮目标 rpm：
 *
 *   ChassisCommand_t（队列）
 *          -> 格式/有限性/状态机安全检查
 *          -> 状态机（速度、轨迹、停车、故障）
 *          -> 限速、斜坡或七段 S 曲线
 *          -> ChassisMecanum_Inverse()
 *          -> FL/FR/RL/RR 目标 rpm（由 chassis_task 交给 Port 层）
 *
 * 反馈方向相反：Port 层缓存 C620 反馈，任务层读出 rpm 后调用
 * ChassisControl_UpdateMotorFeedback()；这里做正运动学、更新实际速度并
 * 积分里程计。CAN 回调保持轻量，不能在中断中调用本文件的完整控制流程。
 * ======================================================================
 */

/*
 * 低于这些阈值即认为底盘已经停稳。线速度单位 m/s，角速度单位 rad/s。
 * STOPPING 状态不会立刻切换 IDLE，而是先经过斜坡限制器，避免速度指令
 * 在非零时骤然归零；达到阈值后才结束停车过程。
 */
#define CHASSIS_CONTROL_STOP_EPSILON_LINEAR   (0.005f)
#define CHASSIS_CONTROL_STOP_EPSILON_ANGULAR  (0.01f)
#define CHASSIS_CONTROL_PI                    (3.14159265358979323846f)
#define CHASSIS_CONTROL_TWO_PI                (2.0f * CHASSIS_CONTROL_PI)

static uint8_t ChassisControl_IsFinite(float value)
{
    /* NaN 不等于自身；FLT_MAX 上下界可排除正/负无穷大。 */
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static float ChassisControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ChassisControl_NormalizeAngle(float angle_rad)
{
    /* 将角度规范到 [-pi, pi]，使绝对偏航目标默认走较短的转向方向。 */
    while (angle_rad > CHASSIS_CONTROL_PI)
    {
        angle_rad -= CHASSIS_CONTROL_TWO_PI;
    }
    while (angle_rad < -CHASSIS_CONTROL_PI)
    {
        angle_rad += CHASSIS_CONTROL_TWO_PI;
    }
    return angle_rad;
}

/*
 * 按一个共同系数限幅 Vx/Vy/Wz，而不是分别把每一轴截断到上限。
 * 例如原速度 [2, 1, 0] m/s，若 Vx 上限是 1 m/s，则输出 [1, 0.5, 0]；
 * 这样合速度方向保持不变。Wz 同理，因此复合移动时转弯半径也不会被改变。
 */
static RobotBodyVelocity_t ChassisControl_LimitBodyVelocity(
    const RobotBodyVelocity_t *input,
    const ChassisControl_Config_t *config)
{
    RobotBodyVelocity_t output = *input;
    float scale = 1.0f;
    float candidate;

    if (ChassisControl_Abs(input->vx_mps) > config->max_vx_mps)
    {
        scale = config->max_vx_mps / ChassisControl_Abs(input->vx_mps);
    }
    if (ChassisControl_Abs(input->vy_mps) > config->max_vy_mps)
    {
        candidate = config->max_vy_mps / ChassisControl_Abs(input->vy_mps);
        if (candidate < scale)
        {
            scale = candidate;
        }
    }
    if (ChassisControl_Abs(input->wz_radps) > config->max_wz_radps)
    {
        candidate = config->max_wz_radps / ChassisControl_Abs(input->wz_radps);
        if (candidate < scale)
        {
            scale = candidate;
        }
    }

    output.vx_mps *= scale;
    output.vy_mps *= scale;
    output.wz_radps *= scale;
    return output;
}

static uint8_t ChassisControl_HasElapsed(
    uint32_t now_ms,
    uint32_t start_ms,
    uint32_t timeout_ms)
{
    /* 无符号减法可自然处理 HAL tick 溢出，只要超时窗口远小于 2^32 ms。 */
    return (uint8_t)((timeout_ms > 0U) &&
                     ((uint32_t)(now_ms - start_ms) > timeout_ms));
}

static RobotBodyVelocity_t ChassisControl_ZeroVelocity(void)
{
    RobotBodyVelocity_t velocity = {0.0f, 0.0f, 0.0f};
    return velocity;
}

static void ChassisControl_ZeroMotorTargets(ChassisControl_t *control)
{
    uint32_t index;

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        control->status.target_motor_rpm[index] = 0.0f;
    }
    /* 静止时没有超速缩放的含义，恢复为 1.0f 便于上位机解释状态。 */
    control->status.motor_scale = 1.0f;
    control->status.commanded_body_velocity = ChassisControl_ZeroVelocity();
}

static void ChassisControl_FillSafeOutput(
    ChassisControl_t *control,
    ChassisControl_Output_t *output)
{
    /*
     * 统一的安全输出：即使控制器禁用或故障，任务层仍会拿到一组可发送的
     * 四路 0 rpm，避免下游因“没有新命令”继续保留上一帧非零目标。
     */
    uint32_t index;

    output->send_motor_targets = 1U;
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        output->motor_rpm[index] = 0.0f;
    }
    ChassisControl_ZeroMotorTargets(control);
}

static uint8_t ChassisControl_IsPlannerLimitValid(
    const ChassisControl_PlannerLimit_t *limit)
{
    /* 规划器中的除法和开方依赖全部限值为有限正数。 */
    return (uint8_t)(ChassisControl_IsFinite(limit->max_acceleration) &&
                     ChassisControl_IsFinite(limit->max_velocity) &&
                     ChassisControl_IsFinite(limit->max_jerk) &&
                     (limit->max_acceleration > 0.0f) &&
                     (limit->max_velocity > 0.0f) &&
                     (limit->max_jerk > 0.0f));
}

static uint8_t ChassisControl_IsConfigValid(
    const ChassisControl_Config_t *config)
{
    /*
     * 此处验证控制层参数；轮径、减速比、轮距和方向由
     * ChassisMecanum_Init() 继续验证。任何一层验证失败都会阻止使能。
     */
    if ((!ChassisControl_IsFinite(config->max_vx_mps)) ||
        (!ChassisControl_IsFinite(config->max_vy_mps)) ||
        (!ChassisControl_IsFinite(config->max_wz_radps)) ||
        (!ChassisControl_IsFinite(config->max_vx_accel_mps2)) ||
        (!ChassisControl_IsFinite(config->max_vy_accel_mps2)) ||
        (!ChassisControl_IsFinite(config->max_wz_accel_radps2)))
    {
        return 0U;
    }

    if ((config->max_vx_mps <= 0.0f) ||
        (config->max_vy_mps <= 0.0f) ||
        (config->max_wz_radps <= 0.0f) ||
        (config->max_vx_accel_mps2 <= 0.0f) ||
        (config->max_vy_accel_mps2 <= 0.0f) ||
        (config->max_wz_accel_radps2 <= 0.0f) ||
        (config->command_timeout_ms == 0U) ||
        (config->feedback_timeout_ms == 0U))
    {
        return 0U;
    }

    return (uint8_t)(ChassisControl_IsPlannerLimitValid(
                         &config->planner_translation) &&
                     ChassisControl_IsPlannerLimitValid(&config->planner_yaw));
}

static uint8_t ChassisControl_IsVelocityValid(
    const RobotBodyVelocity_t *velocity)
{
    return (uint8_t)(ChassisControl_IsFinite(velocity->vx_mps) &&
                     ChassisControl_IsFinite(velocity->vy_mps) &&
                     ChassisControl_IsFinite(velocity->wz_radps));
}

static uint8_t ChassisControl_IsPoseValid(const RobotPose2D_t *pose)
{
    return (uint8_t)(ChassisControl_IsFinite(pose->x_m) &&
                     ChassisControl_IsFinite(pose->y_m) &&
                     ChassisControl_IsFinite(pose->yaw_rad));
}

static uint8_t ChassisControl_IsCommandPayloadValid(
    const ChassisCommand_t *command)
{
    /*
     * 这里只验证协议语义和浮点数合法性，不检查“是否允许在当前状态执行”。
     * 后者依赖状态机，应在 ChassisControl_SubmitCommand() 中判断。
     */
    if ((command->source <= CHASSIS_SOURCE_NONE) ||
        (command->source > CHASSIS_SOURCE_SAFETY) ||
        (command->type > CHASSIS_COMMAND_MOVE_ABSOLUTE) ||
        (command->frame > CHASSIS_FRAME_ODOM))
    {
        return 0U;
    }

    if (command->type == CHASSIS_COMMAND_BODY_VELOCITY)
    {
        return ChassisControl_IsVelocityValid(&command->payload.body_velocity);
    }

    if ((command->type == CHASSIS_COMMAND_MOVE_RELATIVE) ||
        (command->type == CHASSIS_COMMAND_MOVE_ABSOLUTE))
    {
        return ChassisControl_IsPoseValid(&command->payload.pose);
    }

    return 1U;
}

static void ChassisControl_ResetVelocityMemory(ChassisControl_t *control)
{
    /* 清除速度模式的目标和斜坡历史，防止重新使能后继续沿用旧速度。 */
    control->velocity_target = ChassisControl_ZeroVelocity();
    control->status.requested_body_velocity = ChassisControl_ZeroVelocity();
    ChassisMecanum_SlewLimiterReset(&control->slew_limiter, NULL);
}

static void ChassisControl_EnterFault(
    ChassisControl_t *control,
    uint32_t fault_flags)
{
    /*
     * 故障位采用“累加锁存”而非覆盖，便于调试时同时看到根因和后续结果。
     * 本函数不试图自动恢复，恢复动作必须经 CLEAR_FAULT 并重新 ENABLE。
     */
    control->status.fault_flags |= fault_flags;
    control->status.state = CHASSIS_CONTROL_STATE_FAULT;
    ChassisControl_ResetVelocityMemory(control);
    ChassisControl_ZeroMotorTargets(control);
}

static void ChassisControl_SeedPlanner(
    SpeedPlan_TypeDef *planner,
    float current_velocity)
{
    /*
     * 新轨迹可能打断旧轨迹。把测得的当前速度作为规划器初速度，可减少
     * 轨迹切换造成的速度突变；direction_flag 仅记录这份继承速度的方向。
     */
    planner->a = 0.0f;
    planner->v = ChassisControl_Abs(current_velocity);
    planner->s = 0.0f;
    planner->direction_flag = (current_velocity < 0.0f) ? -1.0f : 1.0f;
}

static void ChassisControl_StartTrajectory(
    ChassisControl_t *control,
    const ChassisCommand_t *command)
{
    /*
     * MOVE_RELATIVE 的 pose 既可按 BODY 也可按 ODOM 解释；BODY 相对位移
     * 需先用当前 yaw 旋转到 ODOM 坐标。MOVE_ABSOLUTE 只能由 ODOM 目标进入。
     * 以下 cosine/sine 是 BODY <-> ODOM 的二维旋转矩阵。
     */
    const float cosine = cosf(control->status.pose.yaw_rad);
    const float sine = sinf(control->status.pose.yaw_rad);
    RobotPose2D_t target;
    float delta_x;
    float delta_y;
    float velocity_x_odom;
    float velocity_y_odom;
    float velocity_along_path;

    if (command->type == CHASSIS_COMMAND_MOVE_ABSOLUTE)
    {
        target = command->payload.pose;
        target.yaw_rad = control->status.pose.yaw_rad +
            ChassisControl_NormalizeAngle(
                target.yaw_rad - control->status.pose.yaw_rad);
    }
    else
    {
        target = control->status.pose;
        if (command->frame == CHASSIS_FRAME_BODY)
        {
            target.x_m += cosine * command->payload.pose.x_m -
                          sine * command->payload.pose.y_m;
            target.y_m += sine * command->payload.pose.x_m +
                          cosine * command->payload.pose.y_m;
        }
        else
        {
            target.x_m += command->payload.pose.x_m;
            target.y_m += command->payload.pose.y_m;
        }
        target.yaw_rad += command->payload.pose.yaw_rad;
    }

    /* 将二维目标压缩为一条“起点到终点”的直线路径，供单个平移规划器使用。 */
    delta_x = target.x_m - control->status.pose.x_m;
    delta_y = target.y_m - control->status.pose.y_m;
    control->trajectory_distance_m = sqrtf(delta_x * delta_x + delta_y * delta_y);
    if (control->trajectory_distance_m > 0.000001f)
    {
        control->trajectory_direction_x =
            delta_x / control->trajectory_distance_m;
        control->trajectory_direction_y =
            delta_y / control->trajectory_distance_m;
    }
    else
    {
        control->trajectory_direction_x = 0.0f;
        control->trajectory_direction_y = 0.0f;
    }

    /*
     * 规划起步速度应继承当前“沿路径”的实际速度。实际车体速度由轮速反馈
     * 得到，先从 BODY 坐标旋转到 ODOM，再投影到路径方向向量上。
     */
    velocity_x_odom =
        cosine * control->status.actual_body_velocity.vx_mps -
        sine * control->status.actual_body_velocity.vy_mps;
    velocity_y_odom =
        sine * control->status.actual_body_velocity.vx_mps +
        cosine * control->status.actual_body_velocity.vy_mps;
    velocity_along_path =
        velocity_x_odom * control->trajectory_direction_x +
        velocity_y_odom * control->trajectory_direction_y;

    /* 两条 S 曲线分别负责平移距离和偏航角，随后由 Step 周期推进。 */
    control->trajectory_start_pose = control->status.pose;
    control->status.target_pose = target;
    ChassisControl_SeedPlanner(
        &control->planner_translation,
        velocity_along_path);
    ChassisControl_SeedPlanner(
        &control->planner_yaw,
        control->status.actual_body_velocity.wz_radps);
    control->planner_translation.state = init;
    control->planner_yaw.state = init;
    control->status.state = CHASSIS_CONTROL_STATE_TRAJECTORY;
}

static void ChassisControl_UpdateOdometry(
    ChassisControl_t *control,
    uint32_t now_ms,
    float delta_time_s)
{
    /*
     * 没有新鲜反馈时不积分，避免电机离线后以旧速度不断累加出虚假位姿。
     * 这是纯轮速里程计，打滑时仍会漂移；本模块未融合 IMU/视觉校正。
     */
    float cosine;
    float sine;

    if ((control->status.feedback_valid == 0U) ||
        ChassisControl_HasElapsed(
            now_ms,
            control->status.last_feedback_ms,
            control->config.feedback_timeout_ms))
    {
        return;
    }

    cosine = cosf(control->status.pose.yaw_rad);
    sine = sinf(control->status.pose.yaw_rad);

    /* 将 BODY 速度旋转到 ODOM 后做一阶积分，yaw 用实际 Wz 积分。 */
    control->status.pose.x_m +=
        (cosine * control->status.actual_body_velocity.vx_mps -
         sine * control->status.actual_body_velocity.vy_mps) * delta_time_s;
    control->status.pose.y_m +=
        (sine * control->status.actual_body_velocity.vx_mps +
         cosine * control->status.actual_body_velocity.vy_mps) * delta_time_s;
    control->status.pose.yaw_rad +=
        control->status.actual_body_velocity.wz_radps * delta_time_s;
}

static RobotBodyVelocity_t ChassisControl_CalculateTrajectoryVelocity(
    ChassisControl_t *control)
{
    /*
     * 此函数的平移规划始终在 ODOM 坐标中沿直线推进：先计算已完成的路径
     * 投影距离 progress_m，再得到 ODOM 速度，最后变换回麦轮逆解所需的
     * BODY 速度。这样不会因车体当前朝向变化而改变全局直线路径。
     */
    RobotBodyVelocity_t velocity;
    float velocity_x_odom;
    float velocity_y_odom;
    float progress_m;
    const float cosine = cosf(control->status.pose.yaw_rad);
    const float sine = sinf(control->status.pose.yaw_rad);

    progress_m =
        (control->status.pose.x_m - control->trajectory_start_pose.x_m) *
            control->trajectory_direction_x +
        (control->status.pose.y_m - control->trajectory_start_pose.y_m) *
            control->trajectory_direction_y;

    /* 两个规划器的 v 是无符号速度大小，direction_flag 决定最终正负方向。 */
    SpeedPlanUpdate(
        &control->planner_translation,
        progress_m,
        control->trajectory_distance_m);
    SpeedPlanUpdate(
        &control->planner_yaw,
        control->status.pose.yaw_rad,
        control->status.target_pose.yaw_rad);

    velocity_x_odom = control->trajectory_direction_x *
        control->planner_translation.direction_flag *
        control->planner_translation.v;
    velocity_y_odom = control->trajectory_direction_y *
        control->planner_translation.direction_flag *
        control->planner_translation.v;

    /* 规划器在 ODOM 坐标系工作；麦轮逆解需要 BODY 坐标系速度。 */
    velocity.vx_mps = cosine * velocity_x_odom + sine * velocity_y_odom;
    velocity.vy_mps = -sine * velocity_x_odom + cosine * velocity_y_odom;
    velocity.wz_radps =
        control->planner_yaw.direction_flag * control->planner_yaw.v;

    return ChassisControl_LimitBodyVelocity(&velocity, &control->config);
}

static uint8_t ChassisControl_IsStopped(
    const RobotBodyVelocity_t *velocity)
{
    /* 三轴都低于阈值才可认为停稳，避免仅停止平移但仍在旋转时过早切 IDLE。 */
    return (uint8_t)(
        (ChassisControl_Abs(velocity->vx_mps) <=
         CHASSIS_CONTROL_STOP_EPSILON_LINEAR) &&
        (ChassisControl_Abs(velocity->vy_mps) <=
         CHASSIS_CONTROL_STOP_EPSILON_LINEAR) &&
        (ChassisControl_Abs(velocity->wz_radps) <=
         CHASSIS_CONTROL_STOP_EPSILON_ANGULAR));
}

ChassisControl_Result_t ChassisControl_Init(
    ChassisControl_t *control,
    const ChassisControl_Config_t *config,
    uint32_t now_ms)
{
    /*
     * 初始化顺序：清空旧状态 -> 验证控制参数 -> 初始化运动学 -> 初始化斜坡
     * -> 初始化两条 S 曲线。成功后刻意保持 DISABLED，必须由上层明确 ENABLE。
     */
    ChassisMecanum_Status_t mecanum_status;

    if ((control == NULL) || (config == NULL))
    {
        return CHASSIS_CONTROL_NULL_POINTER;
    }

    memset(control, 0, sizeof(*control));
    control->status.state = CHASSIS_CONTROL_STATE_UNINITIALIZED;

    if (!ChassisControl_IsConfigValid(config))
    {
        control->status.fault_flags = CHASSIS_FAULT_CONFIG;
        return CHASSIS_CONTROL_INVALID_CONFIG;
    }

    mecanum_status = ChassisMecanum_Init(&control->mecanum, &config->mecanum);
    if (mecanum_status != CHASSIS_MECANUM_STATUS_OK)
    {
        control->status.fault_flags = CHASSIS_FAULT_CONFIG;
        return CHASSIS_CONTROL_INVALID_CONFIG;
    }

    mecanum_status = ChassisMecanum_SlewLimiterInit(
        &control->slew_limiter,
        config->max_vx_accel_mps2,
        config->max_vy_accel_mps2,
        config->max_wz_accel_radps2);
    if (mecanum_status != CHASSIS_MECANUM_STATUS_OK)
    {
        control->status.fault_flags = CHASSIS_FAULT_CONFIG;
        return CHASSIS_CONTROL_INVALID_CONFIG;
    }

    control->config = *config;
    SpeedPlanInit(
        &control->planner_translation,
        config->planner_translation.max_acceleration,
        config->planner_translation.max_velocity,
        config->planner_translation.max_jerk);
    SpeedPlanInit(
        &control->planner_yaw,
        config->planner_yaw.max_acceleration,
        config->planner_yaw.max_velocity,
        config->planner_yaw.max_jerk);

    /* 初始不自动使能，防止系统启动即向尚未确认在线的电调下发目标。 */
    control->status.state = CHASSIS_CONTROL_STATE_DISABLED;
    control->status.active_source = CHASSIS_SOURCE_NONE;
    control->status.last_command_ms = now_ms;
    control->status.last_feedback_ms = now_ms;
    control->status.motor_scale = 1.0f;
    control->status.initialized = 1U;
    return CHASSIS_CONTROL_OK;
}

ChassisControl_Result_t ChassisControl_SubmitCommand(
    ChassisControl_t *control,
    const ChassisCommand_t *command,
    uint32_t now_ms)
{
    /*
     * 上位机/总状态机已完成“谁拥有控制权、命令先后关系、单帧业务有效期”的
     * 仲裁。本函数不再基于 source、sequence、issued_at_ms 或 valid_for_ms
     * 拒绝命令，避免同一策略在两端重复实现并产生不一致。
     *
     * 下位机仍必须校验枚举范围、浮点有限性及当前状态机，因为这些直接关系
     * 到内存安全、执行器安全和硬件故障处理，不能依赖通信链路的正确性。
     */

    if ((control == NULL) || (command == NULL))
    {
        return CHASSIS_CONTROL_NULL_POINTER;
    }
    if (control->status.initialized == 0U)
    {
        return CHASSIS_CONTROL_NOT_INITIALIZED;
    }
    if (!ChassisControl_IsCommandPayloadValid(command))
    {
        return CHASSIS_CONTROL_INVALID_COMMAND;
    }

    /* ESTOP 立即锁存，之后仅 SAFETY 来源可以 CLEAR_FAULT。 */
    if (command->type == CHASSIS_COMMAND_ESTOP)
    {
        control->status.active_source = command->source;
        control->status.last_sequence = command->sequence;
        control->status.last_command_ms = now_ms;
        ChassisControl_EnterFault(control, CHASSIS_FAULT_ESTOP);
        return CHASSIS_CONTROL_OK;
    }

    /* 清故障只重置软件状态，不会自动重新使能，也不能修复 CAN/供电等硬件问题。 */
    if (command->type == CHASSIS_COMMAND_CLEAR_FAULT)
    {
        if (((control->status.fault_flags & CHASSIS_FAULT_ESTOP) != 0U) &&
            (command->source != CHASSIS_SOURCE_SAFETY))
        {
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        control->status.fault_flags = CHASSIS_FAULT_NONE;
        control->status.state = CHASSIS_CONTROL_STATE_DISABLED;
        control->status.active_source = CHASSIS_SOURCE_NONE;
        control->status.last_sequence = 0U;
        control->status.last_command_ms = now_ms;
        ChassisControl_ResetVelocityMemory(control);
        ChassisControl_ZeroMotorTargets(control);
        return CHASSIS_CONTROL_OK;
    }

    /* DISABLE 直接置零并释放控制权；若希望按加速度平滑停车，应发送 STOP。 */
    if (command->type == CHASSIS_COMMAND_DISABLE)
    {
        control->status.state = CHASSIS_CONTROL_STATE_DISABLED;
        control->status.active_source = CHASSIS_SOURCE_NONE;
        control->status.last_sequence = 0U;
        control->status.last_command_ms = now_ms;
        ChassisControl_ResetVelocityMemory(control);
        ChassisControl_ZeroMotorTargets(control);
        return CHASSIS_CONTROL_OK;
    }

    control->status.active_source = command->source;
    control->status.last_sequence = command->sequence;
    control->status.last_command_ms = now_ms;

    switch (command->type)
    {
    case CHASSIS_COMMAND_HEARTBEAT:
        /* 仅刷新 last_command_ms，维持当前模式与控制权。 */
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_ENABLE:
        /*
         * ENABLE 前再次检查故障和反馈。只有电机反馈存在且未超时，才允许
         * 从 DISABLED 进入 IDLE；这能避免盲目使能离线或接线错误的电调。
         */
        if (control->status.fault_flags != CHASSIS_FAULT_NONE)
        {
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        if ((control->config.require_motor_feedback != 0U) &&
            ((control->status.feedback_valid == 0U) ||
             ChassisControl_HasElapsed(
                now_ms,
                control->status.last_feedback_ms,
                control->config.feedback_timeout_ms)))
        {
            ChassisControl_EnterFault(
                control,
                CHASSIS_FAULT_FEEDBACK_TIMEOUT);
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        ChassisControl_ResetVelocityMemory(control);
        control->status.state = CHASSIS_CONTROL_STATE_IDLE;
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_STOP:
        /* STOP 清空速度目标后由 Step 的 SlewLimiter 执行受限减速。 */
        if ((control->status.state != CHASSIS_CONTROL_STATE_DISABLED) &&
            (control->status.state != CHASSIS_CONTROL_STATE_FAULT))
        {
            control->velocity_target = ChassisControl_ZeroVelocity();
            control->status.requested_body_velocity =
                ChassisControl_ZeroVelocity();
            control->status.state = CHASSIS_CONTROL_STATE_STOPPING;
        }
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_BODY_VELOCITY:
        /* 速度命令只接受 BODY 坐标系，防止上层误把 ODOM 速度送进麦轮逆解。 */
        if ((command->frame != CHASSIS_FRAME_BODY) ||
            (control->status.state == CHASSIS_CONTROL_STATE_DISABLED) ||
            (control->status.state == CHASSIS_CONTROL_STATE_FAULT))
        {
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        control->velocity_target = ChassisControl_LimitBodyVelocity(
            &command->payload.body_velocity,
            &control->config);
        control->status.requested_body_velocity = control->velocity_target;
        control->status.state = CHASSIS_CONTROL_STATE_VELOCITY;
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_MOVE_RELATIVE:
        /* 相对位姿可按 BODY 或 ODOM 解释，转换细节在 StartTrajectory。 */
        if ((control->status.state == CHASSIS_CONTROL_STATE_DISABLED) ||
            (control->status.state == CHASSIS_CONTROL_STATE_FAULT))
        {
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        ChassisControl_StartTrajectory(control, command);
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_MOVE_ABSOLUTE:
        /* 绝对位姿必须是 ODOM 坐标，否则“绝对”的基准没有确定含义。 */
        if ((command->frame != CHASSIS_FRAME_ODOM) ||
            (control->status.state == CHASSIS_CONTROL_STATE_DISABLED) ||
            (control->status.state == CHASSIS_CONTROL_STATE_FAULT))
        {
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        ChassisControl_StartTrajectory(control, command);
        return CHASSIS_CONTROL_OK;

    default:
        return CHASSIS_CONTROL_INVALID_COMMAND;
    }
}

ChassisControl_Result_t ChassisControl_UpdateMotorFeedback(
    ChassisControl_t *control,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t feedback_time_ms)
{
    /*
     * FDCAN 回调不直接进这里。Port 层先缓存每个 C620 的反馈，控制任务在
     * 普通线程上下文复制 rpm 后调用本函数，避免在中断内做浮点解算和状态机。
     */
    ChassisMecanum_BodyVelocity_t body_velocity;
    ChassisMecanum_Status_t status;
    uint32_t index;

    if ((control == NULL) || (motor_rpm == NULL))
    {
        return CHASSIS_CONTROL_NULL_POINTER;
    }
    if (control->status.initialized == 0U)
    {
        return CHASSIS_CONTROL_NOT_INITIALIZED;
    }

    /* 将电机轴 rpm 还原为底盘 BODY 速度；失败即视为算法/数据异常并停车。 */
    status = ChassisMecanum_Forward(
        &control->mecanum,
        motor_rpm,
        &body_velocity);
    if (status != CHASSIS_MECANUM_STATUS_OK)
    {
        ChassisControl_EnterFault(control, CHASSIS_FAULT_ALGORITHM);
        return CHASSIS_CONTROL_ALGORITHM_ERROR;
    }

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        control->status.feedback_motor_rpm[index] = motor_rpm[index];
    }
    control->status.actual_body_velocity.vx_mps = body_velocity.vx_mps;
    control->status.actual_body_velocity.vy_mps = body_velocity.vy_mps;
    control->status.actual_body_velocity.wz_radps = body_velocity.wz_radps;
    control->status.last_feedback_ms = feedback_time_ms;
    control->status.feedback_valid = 1U;
    return CHASSIS_CONTROL_OK;
}

ChassisControl_Result_t ChassisControl_Step(
    ChassisControl_t *control,
    uint32_t now_ms,
    float delta_time_s,
    ChassisControl_Output_t *output)
{
    /*
     * 每个固定周期调用一次的控制核心。执行顺序刻意固定：
     * 1. 更新里程计；2. 检查命令与反馈超时；3. 根据状态得到 BODY 速度；
     * 4. 麦轮逆解；5. 输出四路 rpm。任一步异常都转换为可立即发送的零 rpm。
     */
    ChassisMecanum_BodyVelocity_t mecanum_velocity;
    ChassisMecanum_MotorCommand_t motor_command;
    ChassisMecanum_Status_t mecanum_status;
    RobotBodyVelocity_t command_velocity;
    uint32_t index;

    if ((control == NULL) || (output == NULL))
    {
        return CHASSIS_CONTROL_NULL_POINTER;
    }
    if (control->status.initialized == 0U)
    {
        return CHASSIS_CONTROL_NOT_INITIALIZED;
    }
    if ((!ChassisControl_IsFinite(delta_time_s)) || (delta_time_s <= 0.0f))
    {
        ChassisControl_EnterFault(control, CHASSIS_FAULT_ALGORITHM);
        ChassisControl_FillSafeOutput(control, output);
        return CHASSIS_CONTROL_ALGORITHM_ERROR;
    }

    /* 先用上一周期以来最新的反馈更新位姿，轨迹规划才有正确的进度基准。 */
    ChassisControl_UpdateOdometry(control, now_ms, delta_time_s);

    /* 命令或 HEARTBEAT 超时不是立即 FAULT，而是先进入可控的 STOPPING。 */
    if ((control->status.state != CHASSIS_CONTROL_STATE_DISABLED) &&
        (control->status.state != CHASSIS_CONTROL_STATE_FAULT) &&
        ChassisControl_HasElapsed(
            now_ms,
            control->status.last_command_ms,
            control->config.command_timeout_ms))
    {
        control->status.fault_flags |= CHASSIS_FAULT_COMMAND_TIMEOUT;
        control->velocity_target = ChassisControl_ZeroVelocity();
        control->status.state = CHASSIS_CONTROL_STATE_STOPPING;
    }

    /* 反馈超时则视为关键安全故障：无法确认实际电机状态，必须立即归零锁存。 */
    if ((control->config.require_motor_feedback != 0U) &&
        (control->status.state != CHASSIS_CONTROL_STATE_DISABLED) &&
        (control->status.state != CHASSIS_CONTROL_STATE_FAULT) &&
        ((control->status.feedback_valid == 0U) ||
         ChassisControl_HasElapsed(
            now_ms,
            control->status.last_feedback_ms,
            control->config.feedback_timeout_ms)))
    {
        ChassisControl_EnterFault(
            control,
            CHASSIS_FAULT_FEEDBACK_TIMEOUT);
    }

    if ((control->status.state == CHASSIS_CONTROL_STATE_DISABLED) ||
        (control->status.state == CHASSIS_CONTROL_STATE_FAULT))
    {
        ChassisControl_FillSafeOutput(control, output);
        return CHASSIS_CONTROL_OK;
    }

    command_velocity = ChassisControl_ZeroVelocity();
    switch (control->status.state)
    {
    case CHASSIS_CONTROL_STATE_IDLE:
        /* 空闲时清除斜坡历史，下一次速度命令从静止平滑起步。 */
        ChassisMecanum_SlewLimiterReset(&control->slew_limiter, NULL);
        break;

    case CHASSIS_CONTROL_STATE_VELOCITY:
    {
        ChassisMecanum_BodyVelocity_t target_velocity;
        ChassisMecanum_BodyVelocity_t limited_velocity;

        target_velocity.vx_mps = control->velocity_target.vx_mps;
        target_velocity.vy_mps = control->velocity_target.vy_mps;
        target_velocity.wz_radps = control->velocity_target.wz_radps;
        /* 速度模式只做一阶加速度限制，不调用位置规划器。 */
        mecanum_status = ChassisMecanum_SlewLimiterStep(
            &control->slew_limiter,
            &target_velocity,
            delta_time_s,
            &limited_velocity);
        if (mecanum_status != CHASSIS_MECANUM_STATUS_OK)
        {
            ChassisControl_EnterFault(control, CHASSIS_FAULT_ALGORITHM);
            ChassisControl_FillSafeOutput(control, output);
            return CHASSIS_CONTROL_ALGORITHM_ERROR;
        }
        command_velocity.vx_mps = limited_velocity.vx_mps;
        command_velocity.vy_mps = limited_velocity.vy_mps;
        command_velocity.wz_radps = limited_velocity.wz_radps;
        break;
    }

    case CHASSIS_CONTROL_STATE_TRAJECTORY:
    {
        ChassisMecanum_BodyVelocity_t trajectory_velocity;

        command_velocity = ChassisControl_CalculateTrajectoryVelocity(control);
        control->status.requested_body_velocity = command_velocity;
        /* 平移与偏航都到位时才结束轨迹，允许“原地转完”或“走完再对准”。 */
        if ((control->planner_translation.state == idle) &&
            (control->planner_yaw.state == idle))
        {
            command_velocity = ChassisControl_ZeroVelocity();
            control->status.state = CHASSIS_CONTROL_STATE_IDLE;
            ChassisMecanum_SlewLimiterReset(&control->slew_limiter, NULL);
        }
        else
        {
            trajectory_velocity.vx_mps = command_velocity.vx_mps;
            trajectory_velocity.vy_mps = command_velocity.vy_mps;
            trajectory_velocity.wz_radps = command_velocity.wz_radps;
            ChassisMecanum_SlewLimiterReset(
                &control->slew_limiter,
                &trajectory_velocity);
        }
        break;
    }

    case CHASSIS_CONTROL_STATE_STOPPING:
    {
        const ChassisMecanum_BodyVelocity_t zero = {0.0f, 0.0f, 0.0f};
        ChassisMecanum_BodyVelocity_t limited_velocity;
        mecanum_status = ChassisMecanum_SlewLimiterStep(
            &control->slew_limiter,
            &zero,
            delta_time_s,
            &limited_velocity);
        if (mecanum_status != CHASSIS_MECANUM_STATUS_OK)
        {
            ChassisControl_EnterFault(control, CHASSIS_FAULT_ALGORITHM);
            ChassisControl_FillSafeOutput(control, output);
            return CHASSIS_CONTROL_ALGORITHM_ERROR;
        }
        command_velocity.vx_mps = limited_velocity.vx_mps;
        command_velocity.vy_mps = limited_velocity.vy_mps;
        command_velocity.wz_radps = limited_velocity.wz_radps;
        /* 停稳后：无故障则回 IDLE；有超时故障则保留 FAULT 等待人工处理。 */
        if (ChassisControl_IsStopped(&command_velocity))
        {
            command_velocity = ChassisControl_ZeroVelocity();
            control->status.state =
                (control->status.fault_flags == CHASSIS_FAULT_NONE) ?
                CHASSIS_CONTROL_STATE_IDLE : CHASSIS_CONTROL_STATE_FAULT;
        }
        break;
    }

    default:
        ChassisControl_EnterFault(control, CHASSIS_FAULT_ALGORITHM);
        ChassisControl_FillSafeOutput(control, output);
        return CHASSIS_CONTROL_ALGORITHM_ERROR;
    }

    /* RobotBodyVelocity_t 与运动学速度结构字段语义相同，这里显式复制便于检查单位。 */
    mecanum_velocity.vx_mps = command_velocity.vx_mps;
    mecanum_velocity.vy_mps = command_velocity.vy_mps;
    mecanum_velocity.wz_radps = command_velocity.wz_radps;
    mecanum_status = ChassisMecanum_Inverse(
        &control->mecanum,
        &mecanum_velocity,
        &motor_command);
    if (mecanum_status != CHASSIS_MECANUM_STATUS_OK)
    {
        ChassisControl_EnterFault(control, CHASSIS_FAULT_ALGORITHM);
        ChassisControl_FillSafeOutput(control, output);
        return CHASSIS_CONTROL_ALGORITHM_ERROR;
    }

    /* 保存本周期最终命令和四轮 rpm，供状态快照、日志和上位机诊断使用。 */
    control->status.commanded_body_velocity = command_velocity;
    control->status.motor_scale = motor_command.scale;
    output->send_motor_targets = 1U;
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        output->motor_rpm[index] = motor_command.motor_rpm[index];
        control->status.target_motor_rpm[index] = motor_command.motor_rpm[index];
    }

    return CHASSIS_CONTROL_OK;
}

void ChassisControl_SetExternalFault(
    ChassisControl_t *control,
    uint32_t fault_flags)
{
    /* Port 层故障不应伪造为普通命令；统一由此入口锁存并生成安全停车状态。 */
    if ((control == NULL) || (control->status.initialized == 0U))
    {
        return;
    }
    if (fault_flags == CHASSIS_FAULT_NONE)
    {
        return;
    }
    ChassisControl_EnterFault(control, fault_flags);
}

void ChassisControl_GetStatus(
    const ChassisControl_t *control,
    ChassisControl_Status_t *status)
{
    /* 本函数仅复制控制器内部快照；跨任务同步由 chassis_task 的 Mutex 负责。 */
    if ((control == NULL) || (status == NULL))
    {
        return;
    }
    *status = control->status;
}
