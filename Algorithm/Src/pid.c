#include "pid.h"

static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PID_Clamp(float value, float limit)
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

/* 写入 PID 增益和限幅，并清空控制器运行状态。 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int)
{
    if (pid == 0)
    {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->max_output = PID_Abs(max_out);
    pid->max_integral = PID_Abs(max_int);
    pid->integral_separation_enabled = 0;
    pid->integral_separation_threshold = 0.0f;
}

/* 执行一次位置式 PID 计算，包含积分分离与输出限幅。 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float derivative;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->target = target;
    pid->actual = actual;
    pid->err = target - actual;

    if ((pid->integral_separation_enabled == 0) ||
        (PID_Abs(pid->err) <= PID_Abs(pid->integral_separation_threshold)))
    {
        pid->integral += pid->err;
    }
    pid->integral = PID_Clamp(pid->integral, pid->max_integral);

    derivative = pid->err - pid->last_err;
    pid->output = (pid->Kp * pid->err) +
                  (pid->Ki * pid->integral) +
                  (pid->Kd * derivative);
    pid->output = PID_Clamp(pid->output, pid->max_output);
    pid->last_err = pid->err;

    return pid->output;
}

/* 清除积分、误差和输出状态，保留增益及限幅配置。 */
void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
