#include "chassis_mecanum.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_EPSILON  (0.0001f)

static void Test_AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static ChassisMecanum_t Test_CreateMecanum(float max_motor_rpm)
{
    ChassisMecanum_t mecanum;
    const ChassisMecanum_Config_t config = {
        .wheel_radius_m = 0.05f,
        .half_wheelbase_m = 0.20f,
        .half_track_m = 0.15f,
        .gear_ratio = 10.0f,
        .max_motor_rpm = max_motor_rpm,
        .motor_direction = {1, 1, 1, 1}
    };

    assert(ChassisMecanum_Init(&mecanum, &config) ==
           CHASSIS_MECANUM_STATUS_OK);
    return mecanum;
}

static void Test_PureMotions(void)
{
    ChassisMecanum_t mecanum = Test_CreateMecanum(10000.0f);
    ChassisMecanum_MotorCommand_t output;
    ChassisMecanum_BodyVelocity_t target = {1.0f, 0.0f, 0.0f};

    assert(ChassisMecanum_Inverse(&mecanum, &target, &output) ==
           CHASSIS_MECANUM_STATUS_OK);
    Test_AssertNear(output.motor_rpm[0], output.motor_rpm[1]);
    Test_AssertNear(output.motor_rpm[1], output.motor_rpm[2]);
    Test_AssertNear(output.motor_rpm[2], output.motor_rpm[3]);

    target.vx_mps = 0.0f;
    target.vy_mps = 1.0f;
    assert(ChassisMecanum_Inverse(&mecanum, &target, &output) ==
           CHASSIS_MECANUM_STATUS_OK);
    assert(output.motor_rpm[0] < 0.0f);
    assert(output.motor_rpm[1] > 0.0f);
    assert(output.motor_rpm[2] > 0.0f);
    assert(output.motor_rpm[3] < 0.0f);

    target.vy_mps = 0.0f;
    target.wz_radps = 1.0f;
    assert(ChassisMecanum_Inverse(&mecanum, &target, &output) ==
           CHASSIS_MECANUM_STATUS_OK);
    assert(output.motor_rpm[0] < 0.0f);
    assert(output.motor_rpm[1] > 0.0f);
    assert(output.motor_rpm[2] < 0.0f);
    assert(output.motor_rpm[3] > 0.0f);
}

static void Test_RoundTripAndDirection(void)
{
    ChassisMecanum_t mecanum = Test_CreateMecanum(10000.0f);
    ChassisMecanum_MotorCommand_t command;
    ChassisMecanum_BodyVelocity_t recovered;
    const ChassisMecanum_BodyVelocity_t target = {0.8f, -0.3f, 0.7f};

    mecanum.config.motor_direction[1] = -1;
    mecanum.config.motor_direction[3] = -1;
    assert(ChassisMecanum_Inverse(&mecanum, &target, &command) ==
           CHASSIS_MECANUM_STATUS_OK);
    assert(ChassisMecanum_Forward(&mecanum, command.motor_rpm, &recovered) ==
           CHASSIS_MECANUM_STATUS_OK);

    Test_AssertNear(recovered.vx_mps, target.vx_mps);
    Test_AssertNear(recovered.vy_mps, target.vy_mps);
    Test_AssertNear(recovered.wz_radps, target.wz_radps);
}

static void Test_UniformSaturation(void)
{
    ChassisMecanum_t mecanum = Test_CreateMecanum(1000.0f);
    ChassisMecanum_MotorCommand_t command;
    const ChassisMecanum_BodyVelocity_t target = {1.0f, 0.5f, 1.0f};
    float max_abs_rpm = 0.0f;
    uint32_t index;

    assert(ChassisMecanum_Inverse(&mecanum, &target, &command) ==
           CHASSIS_MECANUM_STATUS_OK);
    assert(command.scale < 1.0f);

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        const float abs_rpm = fabsf(command.motor_rpm[index]);
        if (abs_rpm > max_abs_rpm)
        {
            max_abs_rpm = abs_rpm;
        }
    }
    Test_AssertNear(max_abs_rpm, 1000.0f);
}

static void Test_SlewLimiter(void)
{
    ChassisMecanum_SlewLimiter_t limiter;
    ChassisMecanum_BodyVelocity_t limited;
    const ChassisMecanum_BodyVelocity_t target = {1.0f, -1.0f, 0.5f};

    assert(ChassisMecanum_SlewLimiterInit(&limiter, 2.0f, 1.0f, 3.0f) ==
           CHASSIS_MECANUM_STATUS_OK);
    assert(ChassisMecanum_SlewLimiterStep(&limiter, &target, 0.1f, &limited) ==
           CHASSIS_MECANUM_STATUS_OK);

    Test_AssertNear(limited.vx_mps, 0.2f);
    Test_AssertNear(limited.vy_mps, -0.1f);
    Test_AssertNear(limited.wz_radps, 0.3f);
}

int main(void)
{
    Test_PureMotions();
    Test_RoundTripAndDirection();
    Test_UniformSaturation();
    Test_SlewLimiter();
    puts("chassis_mecanum tests passed");
    return 0;
}
