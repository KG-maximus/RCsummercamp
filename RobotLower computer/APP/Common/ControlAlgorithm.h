#ifndef CONTROL_ALGORITHM_H
#define CONTROL_ALGORITHM_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_out;
    float max_iout;

    float tar;
    float act;
    float error0;
    float error1;
    float error_i;
    float out;
} PID_TypeDef;

typedef struct
{
    PID_TypeDef outer;
    PID_TypeDef inner;
} CascadePID_TypeDef;

void PIDInit(
    PID_TypeDef *pid,
    float kp,
    float ki,
    float kd,
    float max_out,
    float max_iout);

float PIDCalc(PID_TypeDef *pid, float act, float tar);

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
    float inner_max_iout);

float CascadePIDCalc(
    CascadePID_TypeDef *cascade,
    float outer_act,
    float outer_tar,
    float inner_act);

#endif
