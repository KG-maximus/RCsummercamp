#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "StrategyAlogrithm.h"
#include "chassis_mecanum.h"
#include "robot_protocol.h"

#define CHASSIS_FAULT_NONE              (0UL)
#define CHASSIS_FAULT_CONFIG            (1UL << 0)
#define CHASSIS_FAULT_COMMAND_TIMEOUT   (1UL << 1)
#define CHASSIS_FAULT_FEEDBACK_TIMEOUT  (1UL << 2)
#define CHASSIS_FAULT_ALGORITHM         (1UL << 3)
#define CHASSIS_FAULT_PORT_NOT_READY    (1UL << 4)
#define CHASSIS_FAULT_MOTOR_TX          (1UL << 5)
#define CHASSIS_FAULT_ESTOP             (1UL << 6)

typedef enum
{
    CHASSIS_CONTROL_STATE_UNINITIALIZED = 0,
    CHASSIS_CONTROL_STATE_DISABLED,
    CHASSIS_CONTROL_STATE_IDLE,
    CHASSIS_CONTROL_STATE_VELOCITY,
    CHASSIS_CONTROL_STATE_TRAJECTORY,
    CHASSIS_CONTROL_STATE_STOPPING,
    CHASSIS_CONTROL_STATE_FAULT
} ChassisControl_State_t;

typedef enum
{
    CHASSIS_CONTROL_OK = 0,
    CHASSIS_CONTROL_NULL_POINTER,
    CHASSIS_CONTROL_NOT_INITIALIZED,
    CHASSIS_CONTROL_INVALID_CONFIG,
    CHASSIS_CONTROL_INVALID_COMMAND,
    CHASSIS_CONTROL_COMMAND_REJECTED,
    CHASSIS_CONTROL_ALGORITHM_ERROR
} ChassisControl_Result_t;

typedef struct
{
    float max_acceleration;
    float max_velocity;
    float max_jerk;
} ChassisControl_PlannerLimit_t;

typedef struct
{
    ChassisMecanum_Config_t mecanum;
    float max_vx_mps;
    float max_vy_mps;
    float max_wz_radps;
    float max_vx_accel_mps2;
    float max_vy_accel_mps2;
    float max_wz_accel_radps2;
    ChassisControl_PlannerLimit_t planner_translation;
    ChassisControl_PlannerLimit_t planner_yaw;
    uint32_t command_timeout_ms;
    uint32_t feedback_timeout_ms;
    uint8_t require_motor_feedback;
} ChassisControl_Config_t;

typedef struct
{
    ChassisControl_State_t state;
    uint32_t fault_flags;
    ChassisCommandSource_t active_source;
    uint32_t last_sequence;
    uint32_t last_command_ms;
    uint32_t last_feedback_ms;
    RobotPose2D_t pose;
    RobotPose2D_t target_pose;
    RobotBodyVelocity_t requested_body_velocity;
    RobotBodyVelocity_t commanded_body_velocity;
    RobotBodyVelocity_t actual_body_velocity;
    float target_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    float feedback_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    float motor_scale;
    uint8_t feedback_valid;
    uint8_t initialized;
} ChassisControl_Status_t;

typedef struct
{
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    uint8_t send_motor_targets;
} ChassisControl_Output_t;

typedef struct
{
    ChassisControl_Config_t config;
    ChassisMecanum_t mecanum;
    ChassisMecanum_SlewLimiter_t slew_limiter;
    SpeedPlan_TypeDef planner_translation;
    SpeedPlan_TypeDef planner_yaw;
    RobotPose2D_t trajectory_start_pose;
    float trajectory_direction_x;
    float trajectory_direction_y;
    float trajectory_distance_m;
    RobotBodyVelocity_t velocity_target;
    ChassisControl_Status_t status;
} ChassisControl_t;

ChassisControl_Result_t ChassisControl_Init(
    ChassisControl_t *control,
    const ChassisControl_Config_t *config,
    uint32_t now_ms);

ChassisControl_Result_t ChassisControl_SubmitCommand(
    ChassisControl_t *control,
    const ChassisCommand_t *command,
    uint32_t now_ms);

ChassisControl_Result_t ChassisControl_UpdateMotorFeedback(
    ChassisControl_t *control,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t feedback_time_ms);

ChassisControl_Result_t ChassisControl_Step(
    ChassisControl_t *control,
    uint32_t now_ms,
    float delta_time_s,
    ChassisControl_Output_t *output);

void ChassisControl_SetExternalFault(
    ChassisControl_t *control,
    uint32_t fault_flags);

void ChassisControl_GetStatus(
    const ChassisControl_t *control,
    ChassisControl_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
