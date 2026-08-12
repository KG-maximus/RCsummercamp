#include "chassis_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CHASSIS_CONTROL_STOP_EPSILON_LINEAR   (0.005f)
#define CHASSIS_CONTROL_STOP_EPSILON_ANGULAR  (0.01f)
#define CHASSIS_CONTROL_PI                    (3.14159265358979323846f)
#define CHASSIS_CONTROL_TWO_PI                (2.0f * CHASSIS_CONTROL_PI)

static uint8_t ChassisControl_IsFinite(float value)
{
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

/* 保持 Vx/Vy/Wz 的比例，避免单轴截断改变平移方向或转弯半径。 */
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
    control->status.motor_scale = 1.0f;
    control->status.commanded_body_velocity = ChassisControl_ZeroVelocity();
}

static void ChassisControl_FillSafeOutput(
    ChassisControl_t *control,
    ChassisControl_Output_t *output)
{
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

static uint8_t ChassisControl_IsImmediateSafetyCommand(
    ChassisCommandType_t type)
{
    return (uint8_t)((type == CHASSIS_COMMAND_ESTOP) ||
                     (type == CHASSIS_COMMAND_DISABLE) ||
                     (type == CHASSIS_COMMAND_STOP));
}

static void ChassisControl_ResetVelocityMemory(ChassisControl_t *control)
{
    control->velocity_target = ChassisControl_ZeroVelocity();
    control->status.requested_body_velocity = ChassisControl_ZeroVelocity();
    ChassisMecanum_SlewLimiterReset(&control->slew_limiter, NULL);
}

static void ChassisControl_EnterFault(
    ChassisControl_t *control,
    uint32_t fault_flags)
{
    control->status.fault_flags |= fault_flags;
    control->status.state = CHASSIS_CONTROL_STATE_FAULT;
    ChassisControl_ResetVelocityMemory(control);
    ChassisControl_ZeroMotorTargets(control);
}

static void ChassisControl_SeedPlanner(
    SpeedPlan_TypeDef *planner,
    float current_velocity)
{
    planner->a = 0.0f;
    planner->v = ChassisControl_Abs(current_velocity);
    planner->s = 0.0f;
    planner->direction_flag = (current_velocity < 0.0f) ? -1.0f : 1.0f;
}

static void ChassisControl_StartTrajectory(
    ChassisControl_t *control,
    const ChassisCommand_t *command)
{
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

    velocity_x_odom =
        cosine * control->status.actual_body_velocity.vx_mps -
        sine * control->status.actual_body_velocity.vy_mps;
    velocity_y_odom =
        sine * control->status.actual_body_velocity.vx_mps +
        cosine * control->status.actual_body_velocity.vy_mps;
    velocity_along_path =
        velocity_x_odom * control->trajectory_direction_x +
        velocity_y_odom * control->trajectory_direction_y;

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
    uint32_t validity_ms;
    uint8_t owner_alive;

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

    validity_ms = (command->valid_for_ms == 0U) ?
        control->config.command_timeout_ms : command->valid_for_ms;
    if (ChassisControl_HasElapsed(
            now_ms,
            command->issued_at_ms,
            validity_ms))
    {
        return CHASSIS_CONTROL_COMMAND_REJECTED;
    }

    owner_alive = (uint8_t)(
        (control->status.active_source != CHASSIS_SOURCE_NONE) &&
        (!ChassisControl_HasElapsed(
            now_ms,
            control->status.last_command_ms,
            control->config.command_timeout_ms)));

    if ((!ChassisControl_IsImmediateSafetyCommand(command->type)) &&
        owner_alive &&
        (command->source < control->status.active_source))
    {
        return CHASSIS_CONTROL_COMMAND_REJECTED;
    }

    if ((command->source == control->status.active_source) &&
        (command->sequence != 0U) &&
        (control->status.last_sequence != 0U) &&
        ((int32_t)(command->sequence - control->status.last_sequence) <= 0))
    {
        return CHASSIS_CONTROL_COMMAND_REJECTED;
    }

    if (command->type == CHASSIS_COMMAND_ESTOP)
    {
        control->status.active_source = command->source;
        control->status.last_sequence = command->sequence;
        control->status.last_command_ms = now_ms;
        ChassisControl_EnterFault(control, CHASSIS_FAULT_ESTOP);
        return CHASSIS_CONTROL_OK;
    }

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
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_ENABLE:
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
        if ((control->status.state == CHASSIS_CONTROL_STATE_DISABLED) ||
            (control->status.state == CHASSIS_CONTROL_STATE_FAULT))
        {
            return CHASSIS_CONTROL_COMMAND_REJECTED;
        }
        ChassisControl_StartTrajectory(control, command);
        return CHASSIS_CONTROL_OK;

    case CHASSIS_COMMAND_MOVE_ABSOLUTE:
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

    ChassisControl_UpdateOdometry(control, now_ms, delta_time_s);

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
        ChassisMecanum_SlewLimiterReset(&control->slew_limiter, NULL);
        break;

    case CHASSIS_CONTROL_STATE_VELOCITY:
    {
        ChassisMecanum_BodyVelocity_t target_velocity;
        ChassisMecanum_BodyVelocity_t limited_velocity;

        target_velocity.vx_mps = control->velocity_target.vx_mps;
        target_velocity.vy_mps = control->velocity_target.vy_mps;
        target_velocity.wz_radps = control->velocity_target.wz_radps;
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
    if ((control == NULL) || (status == NULL))
    {
        return;
    }
    *status = control->status;
}
