#include "M3508.h"

static int16_t M3508_ClampCurrent(float current)
{
    if (current > (float)M3508_CURRENT_RAW_MAX)
    {
        return M3508_CURRENT_RAW_MAX;
    }
    if (current < -(float)M3508_CURRENT_RAW_MAX)
    {
        return -M3508_CURRENT_RAW_MAX;
    }
    return (int16_t)current;
}

static uint8_t M3508_GroupBaseId(uint16_t ctrl_id)
{
    return (ctrl_id == M3508_CTRL_ID_1TO4) ? 1U : 5U;
}

static void M3508_InitOne(
    M3508_TypeDef *motor,
    uint8_t id,
    const CascadePID_TypeDef *pid_template)
{
    motor->id = id;
    motor->feedback.angle = 0U;
    motor->feedback.speed_rpm = 0;
    motor->feedback.current = 0;
    motor->feedback.temperature = 0U;
    motor->feedback.update_cnt = 0U;
    motor->feedback.last_update_ms = 0U;
    motor->control.speed_target = 0.0f;
    motor->control.current_output = 0;
    if (pid_template != 0)
    {
        motor->control.pid = *pid_template;
    }
    else
    {
        CascadePIDInit(
            &motor->control.pid,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f);
    }
}

void M3508GroupInit(
    M3508Group_TypeDef *group,
    FDCAN_HandleTypeDef *fdcan_handle,
    uint16_t ctrl_id,
    const CascadePID_TypeDef *pid_template)
{
    uint8_t index;
    uint8_t base_id;

    if ((group == 0) || (fdcan_handle == 0) ||
        ((ctrl_id != M3508_CTRL_ID_1TO4) &&
         (ctrl_id != M3508_CTRL_ID_5TO8)))
    {
        return;
    }

    group->fdcan_handle = fdcan_handle;
    group->ctrl_id = ctrl_id;
    base_id = M3508_GroupBaseId(ctrl_id);
    for (index = 0U; index < M3508_GROUP_SIZE; ++index)
    {
        M3508_InitOne(&group->motor[index], (uint8_t)(base_id + index), pid_template);
    }
}

void M3508GroupSetTarget(
    M3508Group_TypeDef *group,
    uint8_t id,
    float speed_target)
{
    uint8_t base_id;

    if (group == 0)
    {
        return;
    }

    base_id = M3508_GroupBaseId(group->ctrl_id);
    if ((id < base_id) || (id >= (uint8_t)(base_id + M3508_GROUP_SIZE)))
    {
        return;
    }
    group->motor[id - base_id].control.speed_target = speed_target;
}

uint8_t M3508GroupParseFeedback(
    M3508Group_TypeDef *group,
    uint32_t std_id,
    const uint8_t rx_data[8],
    uint32_t now_ms)
{
    uint8_t id;
    uint8_t base_id;
    M3508Feedback_TypeDef *feedback;

    if ((group == 0) || (rx_data == 0) ||
        (std_id <= M3508_FEEDBACK_ID_BASE) ||
        (std_id > (M3508_FEEDBACK_ID_BASE + M3508_ID_MAX)))
    {
        return 0U;
    }

    id = (uint8_t)(std_id - M3508_FEEDBACK_ID_BASE);
    base_id = M3508_GroupBaseId(group->ctrl_id);
    if ((id < base_id) || (id >= (uint8_t)(base_id + M3508_GROUP_SIZE)))
    {
        return 0U;
    }

    feedback = &group->motor[id - base_id].feedback;
    feedback->angle = ((uint16_t)rx_data[0] << 8U) | rx_data[1];
    feedback->speed_rpm = (int16_t)(((uint16_t)rx_data[2] << 8U) | rx_data[3]);
    feedback->current = (int16_t)(((uint16_t)rx_data[4] << 8U) | rx_data[5]);
    feedback->temperature = rx_data[6];
    feedback->update_cnt++;
    feedback->last_update_ms = now_ms;
    return 1U;
}

void M3508GroupUpdate(M3508Group_TypeDef *group, uint8_t enabled)
{
    uint8_t index;
    int16_t current[M3508_GROUP_SIZE] = {0, 0, 0, 0};
    uint8_t tx_data[8];

    if ((group == 0) || (group->fdcan_handle == 0))
    {
        return;
    }

    if (enabled != 0U)
    {
        for (index = 0U; index < M3508_GROUP_SIZE; ++index)
        {
            M3508_TypeDef *motor = &group->motor[index];
            current[index] = M3508_ClampCurrent(CascadePIDCalc(
                &motor->control.pid,
                (float)motor->feedback.speed_rpm,
                motor->control.speed_target,
                (float)motor->feedback.current));
            motor->control.current_output = current[index];
        }
    }
    else
    {
        for (index = 0U; index < M3508_GROUP_SIZE; ++index)
        {
            group->motor[index].control.current_output = 0;
        }
    }

    tx_data[0] = (uint8_t)(current[0] >> 8U);
    tx_data[1] = (uint8_t)current[0];
    tx_data[2] = (uint8_t)(current[1] >> 8U);
    tx_data[3] = (uint8_t)current[1];
    tx_data[4] = (uint8_t)(current[2] >> 8U);
    tx_data[5] = (uint8_t)current[2];
    tx_data[6] = (uint8_t)(current[3] >> 8U);
    tx_data[7] = (uint8_t)current[3];
    FDCANSendStandard(group->fdcan_handle, group->ctrl_id, tx_data, 8U);
}
