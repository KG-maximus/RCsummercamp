#include "chassis_mecanum.h"

#include <float.h>
#include <stddef.h>

#define CHASSIS_MECANUM_PI               (3.14159265358979323846f)
#define CHASSIS_MECANUM_RADPS_TO_RPM     (60.0f / (2.0f * CHASSIS_MECANUM_PI))
#define CHASSIS_MECANUM_RPM_TO_RADPS     ((2.0f * CHASSIS_MECANUM_PI) / 60.0f)

static float ChassisMecanum_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ChassisMecanum_ClampDelta(float current, float target, float max_delta)
{
    const float delta = target - current;

    if (delta > max_delta)
    {
        return current + max_delta;
    }
    if (delta < -max_delta)
    {
        return current - max_delta;
    }
    return target;
}

static uint8_t ChassisMecanum_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static uint8_t ChassisMecanum_IsBodyVelocityValid(
    const ChassisMecanum_BodyVelocity_t *velocity)
{
    return (uint8_t)(ChassisMecanum_IsFinite(velocity->vx_mps) &&
                     ChassisMecanum_IsFinite(velocity->vy_mps) &&
                     ChassisMecanum_IsFinite(velocity->wz_radps));
}

static uint8_t ChassisMecanum_IsConfigValid(
    const ChassisMecanum_Config_t *config)
{
    uint32_t index;

    if ((!ChassisMecanum_IsFinite(config->wheel_radius_m)) ||
        (!ChassisMecanum_IsFinite(config->half_wheelbase_m)) ||
        (!ChassisMecanum_IsFinite(config->half_track_m)) ||
        (!ChassisMecanum_IsFinite(config->gear_ratio)) ||
        (!ChassisMecanum_IsFinite(config->max_motor_rpm)))
    {
        return 0U;
    }

    if ((config->wheel_radius_m <= 0.0f) ||
        (config->half_wheelbase_m < 0.0f) ||
        (config->half_track_m < 0.0f) ||
        ((config->half_wheelbase_m + config->half_track_m) <= 0.0f) ||
        (config->gear_ratio <= 0.0f) ||
        (config->max_motor_rpm <= 0.0f))
    {
        return 0U;
    }

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        if ((config->motor_direction[index] != 1) &&
            (config->motor_direction[index] != -1))
        {
            return 0U;
        }
    }

    return 1U;
}

ChassisMecanum_Status_t ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config)
{
    if ((mecanum == NULL) || (config == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (!ChassisMecanum_IsConfigValid(config))
    {
        mecanum->initialized = 0U;
        return CHASSIS_MECANUM_STATUS_INVALID_CONFIG;
    }

    mecanum->config = *config;
    mecanum->initialized = 1U;
    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command)
{
    float wheel_radps[CHASSIS_MECANUM_WHEEL_COUNT];
    float max_abs_rpm = 0.0f;
    float scale = 1.0f;
    float rotation_arm_m;
    float motor_rpm;
    uint32_t index;

    if ((mecanum == NULL) || (body_velocity == NULL) || (motor_command == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (mecanum->initialized == 0U)
    {
        return CHASSIS_MECANUM_STATUS_NOT_INITIALIZED;
    }
    if (!ChassisMecanum_IsBodyVelocityValid(body_velocity))
    {
        return CHASSIS_MECANUM_STATUS_INVALID_INPUT;
    }

    rotation_arm_m = mecanum->config.half_wheelbase_m +
                     mecanum->config.half_track_m;

    /* Standard X-roller layout: +X forward, +Y left, +Wz counter-clockwise. */
    wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] =
        (body_velocity->vx_mps - body_velocity->vy_mps -
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] =
        (body_velocity->vx_mps + body_velocity->vy_mps +
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] =
        (body_velocity->vx_mps + body_velocity->vy_mps -
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT] =
        (body_velocity->vx_mps - body_velocity->vy_mps +
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        motor_rpm = wheel_radps[index] * mecanum->config.gear_ratio *
                    CHASSIS_MECANUM_RADPS_TO_RPM *
                    (float)mecanum->config.motor_direction[index];
        motor_command->motor_rpm[index] = motor_rpm;

        if (ChassisMecanum_Abs(motor_rpm) > max_abs_rpm)
        {
            max_abs_rpm = ChassisMecanum_Abs(motor_rpm);
        }
    }

    if (max_abs_rpm > mecanum->config.max_motor_rpm)
    {
        scale = mecanum->config.max_motor_rpm / max_abs_rpm;
        for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
        {
            motor_command->motor_rpm[index] *= scale;
        }
    }

    motor_command->scale = scale;
    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity)
{
    float wheel_radps[CHASSIS_MECANUM_WHEEL_COUNT];
    float rotation_arm_m;
    uint32_t index;

    if ((mecanum == NULL) || (motor_rpm == NULL) || (body_velocity == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (mecanum->initialized == 0U)
    {
        return CHASSIS_MECANUM_STATUS_NOT_INITIALIZED;
    }

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        if (!ChassisMecanum_IsFinite(motor_rpm[index]))
        {
            return CHASSIS_MECANUM_STATUS_INVALID_INPUT;
        }

        wheel_radps[index] = motor_rpm[index] *
                             (float)mecanum->config.motor_direction[index] *
                             CHASSIS_MECANUM_RPM_TO_RADPS /
                             mecanum->config.gear_ratio;
    }

    rotation_arm_m = mecanum->config.half_wheelbase_m +
                     mecanum->config.half_track_m;

    body_velocity->vx_mps = mecanum->config.wheel_radius_m * 0.25f *
        (wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]);
    body_velocity->vy_mps = mecanum->config.wheel_radius_m * 0.25f *
        (-wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] -
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]);
    body_velocity->wz_radps = mecanum->config.wheel_radius_m *
        (-wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] -
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]) /
        (4.0f * rotation_arm_m);

    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_SlewLimiterInit(
    ChassisMecanum_SlewLimiter_t *limiter,
    float max_vx_accel_mps2,
    float max_vy_accel_mps2,
    float max_wz_accel_radps2)
{
    if (limiter == NULL)
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if ((!ChassisMecanum_IsFinite(max_vx_accel_mps2)) ||
        (!ChassisMecanum_IsFinite(max_vy_accel_mps2)) ||
        (!ChassisMecanum_IsFinite(max_wz_accel_radps2)) ||
        (max_vx_accel_mps2 <= 0.0f) ||
        (max_vy_accel_mps2 <= 0.0f) ||
        (max_wz_accel_radps2 <= 0.0f))
    {
        return CHASSIS_MECANUM_STATUS_INVALID_CONFIG;
    }

    limiter->current.vx_mps = 0.0f;
    limiter->current.vy_mps = 0.0f;
    limiter->current.wz_radps = 0.0f;
    limiter->max_vx_accel_mps2 = max_vx_accel_mps2;
    limiter->max_vy_accel_mps2 = max_vy_accel_mps2;
    limiter->max_wz_accel_radps2 = max_wz_accel_radps2;
    limiter->initialized = 1U;
    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_SlewLimiterStep(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *target,
    float delta_time_s,
    ChassisMecanum_BodyVelocity_t *limited)
{
    if ((limiter == NULL) || (target == NULL) || (limited == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (limiter->initialized == 0U)
    {
        return CHASSIS_MECANUM_STATUS_NOT_INITIALIZED;
    }
    if ((!ChassisMecanum_IsFinite(delta_time_s)) ||
        (delta_time_s <= 0.0f) ||
        (!ChassisMecanum_IsBodyVelocityValid(target)))
    {
        return CHASSIS_MECANUM_STATUS_INVALID_INPUT;
    }

    limiter->current.vx_mps = ChassisMecanum_ClampDelta(
        limiter->current.vx_mps,
        target->vx_mps,
        limiter->max_vx_accel_mps2 * delta_time_s);
    limiter->current.vy_mps = ChassisMecanum_ClampDelta(
        limiter->current.vy_mps,
        target->vy_mps,
        limiter->max_vy_accel_mps2 * delta_time_s);
    limiter->current.wz_radps = ChassisMecanum_ClampDelta(
        limiter->current.wz_radps,
        target->wz_radps,
        limiter->max_wz_accel_radps2 * delta_time_s);

    *limited = limiter->current;
    return CHASSIS_MECANUM_STATUS_OK;
}

void ChassisMecanum_SlewLimiterReset(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *velocity)
{
    if (limiter == NULL)
    {
        return;
    }

    if ((velocity == NULL) || (!ChassisMecanum_IsBodyVelocityValid(velocity)))
    {
        limiter->current.vx_mps = 0.0f;
        limiter->current.vy_mps = 0.0f;
        limiter->current.wz_radps = 0.0f;
        return;
    }

    limiter->current = *velocity;
}
