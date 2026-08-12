#include "ControlAlgorithm.h"

static float PID_LimitAbs(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PIDInit(
    PID_TypeDef *pid,
    float kp,
    float ki,
    float kd,
    float max_out,
    float max_iout)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->tar = 0.0f;
    pid->act = 0.0f;
    pid->error0 = 0.0f;
    pid->error1 = 0.0f;
    pid->error_i = 0.0f;
    pid->out = 0.0f;
}

float PIDCalc(PID_TypeDef *pid, float act, float tar)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    pid->tar = tar;
    pid->act = act;
    pid->error1 = pid->error0;
    pid->error0 = tar - act;
    pid->error_i += pid->ki * pid->error0;
    pid->error_i = PID_LimitAbs(pid->error_i, pid->max_iout);
    pid->out = PID_LimitAbs(
        pid->kp * pid->error0 + pid->error_i +
            pid->kd * (pid->error0 - pid->error1),
        pid->max_out);
    return pid->out;
}

void CascadePIDInit(
    CascadePID_TypeDef *cascade,
    float outer_kp,
    float outer_ki,
    float outer_kd,
    float outer_max_out,
    float outer_max_iout,
    float inner_kp,
    float inner_ki,
    float inner_kd,
    float inner_max_out,
    float inner_max_iout)
{
    if (cascade == 0)
    {
        return;
    }

    PIDInit(
        &cascade->outer,
        outer_kp,
        outer_ki,
        outer_kd,
        outer_max_out,
        outer_max_iout);
    PIDInit(
        &cascade->inner,
        inner_kp,
        inner_ki,
        inner_kd,
        inner_max_out,
        inner_max_iout);
}

float CascadePIDCalc(
    CascadePID_TypeDef *cascade,
    float outer_act,
    float outer_tar,
    float inner_act)
{
    float inner_tar;

    if (cascade == 0)
    {
        return 0.0f;
    }

    inner_tar = PIDCalc(&cascade->outer, outer_act, outer_tar);
    return PIDCalc(&cascade->inner, inner_act, inner_tar);
}
