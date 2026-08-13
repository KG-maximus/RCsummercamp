#include "M3508.h"

static void M3508Init(M3508_TypeDef *motor, uint8_t id)
{
    if (id < M3508_ID_MIN || id > M3508_ID_MAX)
        return;

    motor->id = id;
    motor->feedback.angle = 0;
    motor->feedback.speed_rpm = 0;
    motor->feedback.current = 0;
    motor->feedback.temperature = 0;
    motor->feedback.update_cnt = 0;

    CascadePIDInit(&motor->control.pid,
                   M3508_SPEED_KP, M3508_SPEED_KI, M3508_SPEED_KD,
                   M3508_SPEED_MAX_OUT, M3508_SPEED_MAX_IOUT,
                   M3508_CURRENT_KP, M3508_CURRENT_KI, M3508_CURRENT_KD,
                   M3508_CURRENT_MAX_OUT, M3508_CURRENT_MAX_IOUT);
    motor->control.speed_target = 0.0f;
    motor->control.current_output = 0;
}

static uint8_t M3508FeedbackId(uint32_t std_id)
{
    if (std_id <= M3508_FEEDBACK_ID_BASE || std_id > M3508_FEEDBACK_ID_BASE + M3508_ID_MAX)
        return 0u;

    return (uint8_t)(std_id - M3508_FEEDBACK_ID_BASE);
}

static void M3508ParseFeedback(M3508Feedback_TypeDef *feedback, const uint8_t *rx_data)
{
    feedback->angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    feedback->speed_rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    feedback->current = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    feedback->temperature = rx_data[6];
    feedback->update_cnt++;
}

static void M3508CurrentPack(uint8_t *tx_data, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    tx_data[0] = (uint8_t)(iq1 >> 8);
    tx_data[1] = (uint8_t)(iq1);
    tx_data[2] = (uint8_t)(iq2 >> 8);
    tx_data[3] = (uint8_t)(iq2);
    tx_data[4] = (uint8_t)(iq3 >> 8);
    tx_data[5] = (uint8_t)(iq3);
    tx_data[6] = (uint8_t)(iq4 >> 8);
    tx_data[7] = (uint8_t)(iq4);
}

static int16_t M3508SpeedControlCalc(M3508_TypeDef *motor)
{
    float speed_actual = (float)motor->feedback.speed_rpm;
    float current_actual = (float)motor->feedback.current;

    float current_out = CascadePIDCalc(&motor->control.pid, speed_actual, motor->control.speed_target, current_actual);
    motor->control.current_output = (int16_t)current_out;

    return motor->control.current_output;
}

void M3508GroupInit(M3508Group_TypeDef *group, FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t ctrl_id)
{
    uint8_t base = (ctrl_id == M3508_CTRL_ID_1TO4) ? 1u : 5u;

    group->FDCAN_Handle = FDCAN_Handle;
    group->ctrl_id = ctrl_id;

    for (uint8_t i = 0; i < M3508_GROUP_SIZE; i++)
        M3508Init(&group->motor[i], (uint8_t)(base + i));
}

void M3508GroupSetTarget(M3508Group_TypeDef *group, uint8_t id, float speed_target)
{
    uint8_t base = (group->ctrl_id == M3508_CTRL_ID_1TO4) ? 1u : 5u;

    if (id < base || id >= (uint8_t)(base + M3508_GROUP_SIZE))
        return;

    group->motor[id - base].control.speed_target = speed_target;
}

uint8_t M3508GroupParseFeedback(M3508Group_TypeDef *group, uint32_t std_id, const uint8_t *rx_data)
{
    uint8_t base = (group->ctrl_id == M3508_CTRL_ID_1TO4) ? 1u : 5u;
    uint8_t id = M3508FeedbackId(std_id);

    if (id < base || id >= (uint8_t)(base + M3508_GROUP_SIZE))
        return 0u;

    M3508ParseFeedback(&group->motor[id - base].feedback, rx_data);
    return 1u;
}

void M3508GroupUpdate(M3508Group_TypeDef *group)
{
    int16_t iq[M3508_GROUP_SIZE];
    uint8_t tx_data[8];

    for (uint8_t i = 0; i < M3508_GROUP_SIZE; i++)
        iq[i] = M3508SpeedControlCalc(&group->motor[i]);

    M3508CurrentPack(tx_data, iq[0], iq[1], iq[2], iq[3]);
    FDCANSendStandard(group->FDCAN_Handle, group->ctrl_id, tx_data, 8);
}

