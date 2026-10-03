#ifndef PID_H
#define PID_H

/* PID 模块：提供独立于硬件的离散位置式 PID 计算与状态管理。 */

typedef struct
{
    float Kp;                             /* 比例系数 */
    float Ki;                             /* 积分系数 */
    float Kd;                             /* 微分系数 */
    float target;                         /* 目标值 */
    float actual;                         /* 当前实际值 */
    float err;                            /* 当前误差：target - actual */
    float last_err;                       /* 上一次计算的误差 */
    float integral;                       /* 累积误差 */
    float output;                         /* 最近一次计算的输出 */
    float max_output;                     /* 输出绝对值限幅 */
    float max_integral;                   /* 积分绝对值限幅 */
    int integral_separation_enabled;      /* 积分分离开关，非零表示启用 */
    float integral_separation_threshold; /* 积分分离误差阈值 */
} PID_t;

/* 初始化 PID 参数；max_out 和 max_int 分别为输出与积分的绝对值上限。 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int);

/* 输入目标值和实际值，执行一次位置式 PID 计算并返回限幅后的输出。 */
float PID_Calc(PID_t *pid, float target, float actual);

/* 清除目标、误差、积分和输出等运行状态，保留 PID 参数及限幅配置。 */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
