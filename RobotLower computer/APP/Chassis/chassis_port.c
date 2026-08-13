#include "chassis_port.h"

#include <string.h>

#include "ControlAlgorithm.h"
#include "M3508.h"
#include "bsp_callback.h"
#include "chassis_config.h"
#include "main.h"

/*
 * 此适配器是 FL/FR/RL/RR 车轮顺序与 C620 协议电调 ID 首次相遇的地方。
 * 运动学模块始终独立于 CAN，因此绝不能把 FDCAN 标识符或反馈字节解析搬进
 * chassis_mecanum.c。这里的 ID 数组与 chassis_config.h 的四个宏一一对应。
 */
static M3508Group_TypeDef chassis_motor_group; /* 四个 C620/M3508 的驱动实例及反馈缓存。 */
static uint8_t chassis_port_initialized;       /* 初始化、回调注册和滤波配置是否已成功完成。 */
static uint8_t chassis_motor_enabled;          /* 是否允许 GroupUpdate 根据 PID 输出控制电流。 */

static const uint8_t chassis_motor_id[CHASSIS_MECANUM_WHEEL_COUNT] =
{
    /* 数组下标对应运动学轮序，数组值对应电调 CAN ID。二者均不可随意换位。 */
    CHASSIS_M3508_ID_FL,
    CHASSIS_M3508_ID_FR,
    CHASSIS_M3508_ID_RL,
    CHASSIS_M3508_ID_RR
};

static uint8_t ChassisPort_IsElapsed(
    uint32_t now_ms,
    uint32_t timestamp_ms,
    uint32_t timeout_ms)
{
    /* timestamp 为 0 表示从未收到反馈；无符号相减能处理 HAL tick 自然溢出。 */
    return (uint8_t)((timestamp_ms == 0U) ||
                     ((uint32_t)(now_ms - timestamp_ms) > timeout_ms));
}

static void ChassisPort_FDCANRxHandler(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint32_t std_id,
    const uint8_t data[8])
{
    /*
     * 此函数由 BSP 的 FDCAN 接收回调调用，处于中断/回调上下文。
     * 它只做两件轻量工作：过滤不属于本底盘总线的句柄，并把一帧 8 字节
     * 数据交给 M3508 驱动更新反馈缓存。不得在这里计算运动学、阻塞等待或
     * 发送复杂报文；控制任务会在固定周期读取缓存。
     */
    if (fdcan_handle != CHASSIS_M3508_CAN_HANDLE)
    {
        return;
    }

    (void)M3508GroupParseFeedback(
        &chassis_motor_group,
        std_id,
        data,
        HAL_GetTick());
}

uint8_t ChassisPort_Init(void)
{
    CascadePID_TypeDef pid_template; /* 同一套初始 PID 模板会复制给电机组内各电机。 */

    /* 重复初始化不重复注册回调，直接检查已有反馈是否在线。 */
    if (chassis_port_initialized != 0U)
    {
        return ChassisPort_IsReady();
    }

    /*
     * 先构造级联 PID 模板。当前配置宏默认全为 0，且 PID READY 默认关闭，
     * 因而即使已连接 CAN 也只会保持零电流停机，适合首次观察反馈。
     */
    CascadePIDInit(
        &pid_template,
        CHASSIS_M3508_SPEED_KP,
        CHASSIS_M3508_SPEED_KI,
        CHASSIS_M3508_SPEED_KD,
        CHASSIS_M3508_SPEED_MAX_OUT,
        CHASSIS_M3508_SPEED_MAX_IOUT,
        CHASSIS_M3508_CURRENT_KP,
        CHASSIS_M3508_CURRENT_KI,
        CHASSIS_M3508_CURRENT_KD,
        CHASSIS_M3508_CURRENT_MAX_OUT,
        CHASSIS_M3508_CURRENT_MAX_IOUT);
    /* 初始化四电机驱动组，控制帧通常为 0x200，具体格式封装在 M3508.c。 */
    M3508GroupInit(
        &chassis_motor_group,
        CHASSIS_M3508_CAN_HANDLE,
        CHASSIS_M3508_CONTROL_ID,
        &pid_template);
    /* 注册全局 FDCAN 分发回调失败时不能继续，因为随后无法得到电机反馈。 */
    if (BSPCallback_RegisterFDCANRxHandler(ChassisPort_FDCANRxHandler) == 0U)
    {
        return 0U;
    }
    /*
     * 放行 C620 反馈 ID 范围 [0x201, 0x204]，范围来自实际使用的最小/最大
     * 电调 ID。该函数负责 FDCAN 标准帧过滤器与接收通知的底层配置。
     */
    FDCANStandardInit(
        CHASSIS_M3508_CAN_HANDLE,
        M3508_FEEDBACK_ID_BASE + CHASSIS_M3508_ID_FL,
        M3508_FEEDBACK_ID_BASE + CHASSIS_M3508_ID_RR);

    /* 先明确禁用输出，再调用一次更新下发零电流，保证上电无残留目标。 */
    chassis_motor_enabled = 0U;
    chassis_port_initialized = 1U;
    M3508GroupUpdate(&chassis_motor_group, 0U);
    return 1U;
}

uint8_t ChassisPort_IsReady(void)
{
    uint32_t index;
    uint32_t now_ms;

    /* 未完成回调/驱动初始化时，不允许底盘状态机使能电机。 */
    if (chassis_port_initialized == 0U)
    {
        return 0U;
    }

    /*
     * 对四轮逐个检查：update_cnt=0 表示从未收到帧；last_update_ms 过期表示
     * CAN 断线、电调掉电或 ID/过滤器不匹配。任一轮异常即整个底盘不可就绪。
     */
    now_ms = HAL_GetTick();
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        const M3508Feedback_TypeDef *feedback =
            &chassis_motor_group.motor[index].feedback;

        if ((feedback->update_cnt == 0U) ||
            ChassisPort_IsElapsed(
                now_ms,
                feedback->last_update_ms,
                CHASSIS_FEEDBACK_TIMEOUT_MS))
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t ChassisPort_SendMotorRpm(
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT])
{
    uint32_t index;

    /* 空指针和未初始化都不允许接触驱动对象，避免误发旧目标。 */
    if ((chassis_port_initialized == 0U) || (motor_rpm == 0))
    {
        return 0U;
    }

    /* 将四轮 rpm 按固定轮序写入对应电调的目标缓存。 */
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        M3508GroupSetTarget(
            &chassis_motor_group,
            chassis_motor_id[index],
            motor_rpm[index]);
    }
    /*
     * 最终输出受两个开关共同保护：任务状态允许电机输出，且配置确认 PID
     * 已标定。任一条件为假均以 enabled=0 调用驱动，驱动应发零电流停机帧。
     */
    M3508GroupUpdate(
        &chassis_motor_group,
        (uint8_t)((chassis_motor_enabled != 0U) &&
                  (CHASSIS_M3508_PID_CONFIG_READY != 0U)));
    return 1U;
}

uint8_t ChassisPort_ReadMotorRpm(
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t *feedback_time_ms)
{
    uint32_t index;
    uint32_t latest_update_ms = 0U;

    /*
     * 本函数只复制 M3508 驱动已解析的快照，完全不等待 CAN。读取前通过
     * IsReady 保证四轮都在线且未超时，避免把“部分旧反馈”当完整车体反馈。
     */
    if ((motor_rpm == 0) || (feedback_time_ms == 0) ||
        (ChassisPort_IsReady() == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        const M3508Feedback_TypeDef *feedback =
            &chassis_motor_group.motor[index].feedback;
        motor_rpm[index] = (float)feedback->speed_rpm;
        if (feedback->last_update_ms > latest_update_ms)
        {
            latest_update_ms = feedback->last_update_ms;
        }
    }
    /* 返回最新轮反馈的时间戳，控制器用它判断本组数据的新鲜程度。 */
    *feedback_time_ms = latest_update_ms;
    return 1U;
}

void ChassisPort_SetMotorEnabled(uint8_t enabled)
{
    if (chassis_port_initialized == 0U)
    {
        return;
    }

    /* 统一归一化为 0/1，避免上层传任意非零值造成含义不清。 */
    chassis_motor_enabled = (uint8_t)(enabled != 0U);
    if (chassis_motor_enabled == 0U)
    {
        /* 禁用必须立即发零电流，不能只改软件标志后等待下一控制周期。 */
        M3508GroupUpdate(&chassis_motor_group, 0U);
    }
}
