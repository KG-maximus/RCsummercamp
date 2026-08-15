#include "encoder.h"

#include "bsp_callback.h"
#include "fdcan_common.h"

/* 距离换算使用的圆周率，单独命名以避免无来源的字面量。 */
#define ENCODER_PI    (3.14159265358979323846f)

/*
 * 单轴内部状态。该结构体只在本文件使用，外部模块无法直接修改原始计数或软件原点。
 * raw_position_count 和 feedback_ready 由 FDCAN 中断回调轻量更新；其余字段由
 * Encoder_Update() 在普通任务中使用。
 */
typedef struct
{
    FDCAN_HandleTypeDef *fdcan_handle;
    uint8_t node_id;
    int8_t direction_sign;

    volatile uint32_t raw_position_count;
    volatile uint8_t feedback_ready;
    uint32_t origin_position_count;
} Encoder_AxisTypeDef;

/* 模块唯一运行状态：两个测量轴和对外只读的 x/y 输出。 */
typedef struct
{
    Encoder_AxisTypeDef x_axis;
    Encoder_AxisTypeDef y_axis;
    Encoder_PositionTypeDef position;
} Encoder_ContextTypeDef;

static Encoder_ContextTypeDef encoder_context;

/* 将一条轴的最新位置计数换算为相对软件原点的行进距离。 */
static float Encoder_ConvertDistance(const Encoder_AxisTypeDef *axis)
{
    int32_t relative_count;
    float wheel_circumference_m;

    if (axis->feedback_ready == 0U)
    {
        return 0.0f;
    }

    relative_count = (int32_t)axis->raw_position_count -
                     (int32_t)axis->origin_position_count;
    relative_count *= (int32_t)axis->direction_sign;

    wheel_circumference_m = 2.0f * ENCODER_PI * ENCODER_WHEEL_RADIUS_M;
    return ((float)relative_count * wheel_circumference_m) /
           (float)ENCODER_COUNTS_PER_REVOLUTION;
}

/* 向一只编码器发送 BRT 协议 FUNC=0x01 的“读取多圈位置”请求。 */
static void Encoder_RequestAxisPosition(const Encoder_AxisTypeDef *axis)
{
    uint8_t data[ENCODER_POSITION_REQUEST_LENGTH];

    data[0] = ENCODER_POSITION_REQUEST_LENGTH;
    data[1] = axis->node_id;
    data[2] = ENCODER_CMD_READ_POSITION;
    data[3] = 0U;

    /* 发送失败仅表示本帧未进入 FIFO；下一周期会再次请求，不在这里做额外状态机。 */
    (void)FDCANSendStandardStatus(axis->fdcan_handle, axis->node_id, data,
                                  ENCODER_POSITION_REQUEST_LENGTH);
}

/* FDCAN 公共回调的转发函数，使 BSP 不需要知道编码器协议细节。 */
static void Encoder_CANRxCallback(FDCAN_HandleTypeDef *fdcan_handle,
                                  uint32_t std_id,
                                  const uint8_t data[8])
{
    Encoder_InputFDCANFrame(fdcan_handle, std_id, data);
}

void Encoder_Init(void)
{
    encoder_context.x_axis.fdcan_handle = ENCODER_X_FDCAN_HANDLE;
    encoder_context.x_axis.node_id = ENCODER_X_NODE_ID;
    encoder_context.x_axis.direction_sign = ENCODER_X_DIRECTION_SIGN;
    encoder_context.x_axis.raw_position_count = 0U;
    encoder_context.x_axis.feedback_ready = 0U;
    encoder_context.x_axis.origin_position_count = 0U;

    encoder_context.y_axis.fdcan_handle = ENCODER_Y_FDCAN_HANDLE;
    encoder_context.y_axis.node_id = ENCODER_Y_NODE_ID;
    encoder_context.y_axis.direction_sign = ENCODER_Y_DIRECTION_SIGN;
    encoder_context.y_axis.raw_position_count = 0U;
    encoder_context.y_axis.feedback_ready = 0U;
    encoder_context.y_axis.origin_position_count = 0U;

    encoder_context.position.x_m = 0.0f;
    encoder_context.position.y_m = 0.0f;

    /* 当前工程通过 BSP 回调对接上层 FDCAN；回调内部只转交原始帧。 */
    (void)BSPCallback_RegisterFDCANRxHandler(Encoder_CANRxCallback);
    (void)FDCANStandardInit(ENCODER_X_FDCAN_HANDLE,
                            ENCODER_X_NODE_ID, ENCODER_X_NODE_ID);
    (void)FDCANStandardInit(ENCODER_Y_FDCAN_HANDLE,
                            ENCODER_Y_NODE_ID, ENCODER_Y_NODE_ID);
}

void Encoder_InputFDCANFrame(FDCAN_HandleTypeDef *fdcan_handle,
                             uint32_t std_id,
                             const uint8_t data[8])
{
    Encoder_AxisTypeDef *axis;
    uint32_t position_count;

    /* 先根据 FDCAN 句柄和 CAN ID 找到该帧实际属于 x 轴还是 y 轴。 */
    if ((fdcan_handle == encoder_context.x_axis.fdcan_handle) &&
        (std_id == encoder_context.x_axis.node_id))
    {
        axis = &encoder_context.x_axis;
    }
    else if ((fdcan_handle == encoder_context.y_axis.fdcan_handle) &&
             (std_id == encoder_context.y_axis.node_id))
    {
        axis = &encoder_context.y_axis;
    }
    else
    {
        return;
    }

    /* BRT 位置回复固定为：07 ID 01 POS0 POS1 POS2 POS3，位置计数为小端序。 */
    if ((data[0] != ENCODER_POSITION_REPLY_LENGTH) ||
        (data[1] != axis->node_id) ||
        (data[2] != ENCODER_CMD_READ_POSITION))
    {
        return;
    }

    position_count = ((uint32_t)data[3]) |
                     ((uint32_t)data[4] << 8) |
                     ((uint32_t)data[5] << 16) |
                     ((uint32_t)data[6] << 24);

    axis->raw_position_count = position_count;
    if (axis->feedback_ready == 0U)
    {
        /* 第一帧只建立软件原点，不把编码器上电前的绝对位置算入 x/y。 */
        axis->origin_position_count = position_count;
        axis->feedback_ready = 1U;
    }
}

void Encoder_Update(void)
{
    /* FDCAN 请求和浮点距离换算都在普通任务中完成。 */
    Encoder_RequestAxisPosition(&encoder_context.x_axis);
    Encoder_RequestAxisPosition(&encoder_context.y_axis);

    encoder_context.position.x_m =
        Encoder_ConvertDistance(&encoder_context.x_axis);
    encoder_context.position.y_m =
        Encoder_ConvertDistance(&encoder_context.y_axis);
}

const Encoder_PositionTypeDef *Encoder_GetPosition(void)
{
    return &encoder_context.position;
}
