#include "chassis_mecanum.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/*
 * ======================== 麦克纳姆运动学单元测试 ========================
 *
 * 该文件是为桌面/主机编译准备的断言测试，不是 STM32 固件的 FreeRTOS 任务，
 * 也不应与嵌入式工程现有 main.c 同时链接。它不需要 FDCAN、电调或真车，
 * 只验证 chassis_mecanum.c 中的纯数学关系：
 *   - 纯前进、横移、旋转时的四轮符号关系；
 *   - 逆解后再正解能还原原车体速度；
 *   - 电机 rpm 超限时四轮统一按比例缩放；
 *   - 一阶速度斜坡遵守 a * dt 的最大变化量。
 *
 * 断言失败表示算法结果与预期不一致。真车方向、CAN ID、轮径和减速比仍必须
 * 在 chassis_config.h 和硬件联调中验证，不能只因本测试通过就直接使能电机。
 * =======================================================================
 */

/* 浮点比较容差。计算中包含 pi 和浮点除法，不能使用严格相等比较。 */
#define TEST_EPSILON  (0.0001f)

static void Test_AssertNear(float actual, float expected)
{
    /* 当误差不小于容差时 assert 会中止测试并指出数学关系被破坏。 */
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

static ChassisMecanum_t Test_CreateMecanum(float max_motor_rpm)
{
    ChassisMecanum_t mecanum;
    /*
     * 一套刻意简单、与真实车无关的虚拟参数：半径 0.05 m、L=0.35 m、
     * 减速比 10。测试只关心公式一致性，因此不能把这组值复制进真车配置。
     */
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

    /* 纯前进：Vx 非零、Vy/Wz 为零，四轮目标 rpm 应同号同幅值。 */
    assert(ChassisMecanum_Inverse(&mecanum, &target, &output) ==
           CHASSIS_MECANUM_STATUS_OK);
    Test_AssertNear(output.motor_rpm[0], output.motor_rpm[1]);
    Test_AssertNear(output.motor_rpm[1], output.motor_rpm[2]);
    Test_AssertNear(output.motor_rpm[2], output.motor_rpm[3]);

    /* 纯左移：FL/RR 负，FR/RL 正，符合 X 型麦轮横移符号规律。 */
    target.vx_mps = 0.0f;
    target.vy_mps = 1.0f;
    assert(ChassisMecanum_Inverse(&mecanum, &target, &output) ==
           CHASSIS_MECANUM_STATUS_OK);
    assert(output.motor_rpm[0] < 0.0f);
    assert(output.motor_rpm[1] > 0.0f);
    assert(output.motor_rpm[2] > 0.0f);
    assert(output.motor_rpm[3] < 0.0f);

    /* 纯逆时针旋转：FL/RL 负，FR/RR 正。 */
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

    /*
     * 模拟 FR、RR 两台电机安装方向相反。逆解写出的 rpm 已带方向修正，正解
     * 会撤销同一修正，因此“逆解 -> 正解”仍应还原原始三轴车体速度。
     */
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

    /*
     * 故意把上限设小，迫使至少一轮超速。scale 必须小于 1，且输出的最大
     * 绝对 rpm 恰好等于上限，证明四轮采用共同系数缩放而非逐轮硬截断。
     */
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

    /*
     * 加速度上限分别为 2 m/s^2、1 m/s^2、3 rad/s^2，dt=0.1 s，因此首个
     * 周期的最大变化量分别为 0.2 m/s、0.1 m/s、0.3 rad/s。
     */
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
    /* 主机测试入口：任一 assert 失败都会停止；全部通过后打印成功提示。 */
    Test_PureMotions();
    Test_RoundTripAndDirection();
    Test_UniformSaturation();
    Test_SlewLimiter();
    puts("chassis_mecanum tests passed");
    return 0;
}
