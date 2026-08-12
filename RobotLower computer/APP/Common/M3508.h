#ifndef M3508_H
#define M3508_H

#include <stdint.h>

#include "ControlAlgorithm.h"
#include "fdcan_common.h"

#define M3508_CTRL_ID_1TO4         (0x200U)
#define M3508_CTRL_ID_5TO8         (0x1FFU)
#define M3508_FEEDBACK_ID_BASE     (0x200U)
#define M3508_ID_MIN               (1U)
#define M3508_ID_MAX               (8U)
#define M3508_GROUP_SIZE           (4U)
#define M3508_CURRENT_RAW_MAX      (16384)

typedef struct
{
    uint16_t angle;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    uint32_t update_cnt;
    uint32_t last_update_ms;
} M3508Feedback_TypeDef;

typedef struct
{
    CascadePID_TypeDef pid;
    float speed_target;
    int16_t current_output;
} M3508Control_TypeDef;

typedef struct
{
    uint8_t id;
    M3508Control_TypeDef control;
    M3508Feedback_TypeDef feedback;
} M3508_TypeDef;

typedef struct
{
    uint16_t ctrl_id;
    FDCAN_HandleTypeDef *fdcan_handle;
    M3508_TypeDef motor[M3508_GROUP_SIZE];
} M3508Group_TypeDef;

void M3508GroupInit(
    M3508Group_TypeDef *group,
    FDCAN_HandleTypeDef *fdcan_handle,
    uint16_t ctrl_id,
    const CascadePID_TypeDef *pid_template);

void M3508GroupSetTarget(
    M3508Group_TypeDef *group,
    uint8_t id,
    float speed_target);

uint8_t M3508GroupParseFeedback(
    M3508Group_TypeDef *group,
    uint32_t std_id,
    const uint8_t rx_data[8],
    uint32_t now_ms);

void M3508GroupUpdate(M3508Group_TypeDef *group, uint8_t enabled);

#endif
