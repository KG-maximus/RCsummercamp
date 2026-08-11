#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CHASSIS_MECANUM_WHEEL_COUNT  (4U)

typedef enum
{
    CHASSIS_MECANUM_WHEEL_FRONT_LEFT = 0,
    CHASSIS_MECANUM_WHEEL_FRONT_RIGHT,
    CHASSIS_MECANUM_WHEEL_REAR_LEFT,
    CHASSIS_MECANUM_WHEEL_REAR_RIGHT
} ChassisMecanum_Wheel_t;

typedef enum
{
    CHASSIS_MECANUM_STATUS_OK = 0,
    CHASSIS_MECANUM_STATUS_NULL_POINTER,
    CHASSIS_MECANUM_STATUS_INVALID_CONFIG,
    CHASSIS_MECANUM_STATUS_NOT_INITIALIZED,
    CHASSIS_MECANUM_STATUS_INVALID_INPUT
} ChassisMecanum_Status_t;

typedef struct
{
    float wheel_radius_m;       /* Effective loaded wheel radius. */
    float half_wheelbase_m;     /* Robot center to front/rear wheel axis. */
    float half_track_m;         /* Robot center to left/right wheel center. */
    float gear_ratio;           /* Motor revolutions per wheel revolution. */
    float max_motor_rpm;        /* Limit at the motor shaft, before driver call. */
    int8_t motor_direction[CHASSIS_MECANUM_WHEEL_COUNT]; /* Each entry: +1 or -1. */
} ChassisMecanum_Config_t;

/* Body frame: +X forward, +Y left, +Wz counter-clockwise. */
typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
} ChassisMecanum_BodyVelocity_t;

typedef struct
{
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]; /* FL, FR, RL, RR. */
    float scale; /* 1.0 when unsaturated; all RPM values share this scale. */
} ChassisMecanum_MotorCommand_t;

typedef struct
{
    ChassisMecanum_Config_t config;
    uint8_t initialized;
} ChassisMecanum_t;

typedef struct
{
    ChassisMecanum_BodyVelocity_t current;
    float max_vx_accel_mps2;
    float max_vy_accel_mps2;
    float max_wz_accel_radps2;
    uint8_t initialized;
} ChassisMecanum_SlewLimiter_t;

ChassisMecanum_Status_t ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config);

ChassisMecanum_Status_t ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command);

ChassisMecanum_Status_t ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity);

ChassisMecanum_Status_t ChassisMecanum_SlewLimiterInit(
    ChassisMecanum_SlewLimiter_t *limiter,
    float max_vx_accel_mps2,
    float max_vy_accel_mps2,
    float max_wz_accel_radps2);

ChassisMecanum_Status_t ChassisMecanum_SlewLimiterStep(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *target,
    float delta_time_s,
    ChassisMecanum_BodyVelocity_t *limited);

void ChassisMecanum_SlewLimiterReset(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *velocity);

#ifdef __cplusplus
}
#endif

#endif
