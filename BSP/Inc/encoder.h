#ifndef ENCODER_H
#define ENCODER_H

/* Encoder 模块：读取定时器编码器计数、周期差值和计数速度。 */

#include <stdint.h>
#include "tim.h"

/* 编码器定时器配置占位宏，可在此处或编译选项中替换。 */
#ifndef ENCODER_TIMER
#define ENCODER_TIMER (&htim3)
#endif

#ifndef ENCODER_SAMPLE_TIME_MS
#define ENCODER_SAMPLE_TIME_MS 10U
#endif

#if ENCODER_SAMPLE_TIME_MS <= 0
#error "ENCODER_SAMPLE_TIME_MS must be greater than zero"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 在 MX_TIM3_Init() 后调用；清零计数和软件基准，启动两个编码器通道。 */
void Encoder_Init(void);

/* 读取当前 16 位 CNT，按 int16_t 有符号值返回，不更新差值基准。 */
int16_t Encoder_GetCount(void);

/* 读取相对上次采样的差值并更新基准，不清零硬件 CNT。
 * 支持 16 位正反向回绕；两次采样间实际位移须小于 32768 个计数。 */
int16_t Encoder_GetDelta(void);

/* 返回带方向的计数/秒，无滤波；须每 ENCODER_SAMPLE_TIME_MS 毫秒调用。
 * 内部调用 Encoder_GetDelta()，两者共用基准，勿在同一周期重复采样。
 * TI1/TI2 模式按四倍频后的定时器计数计算。 */
float Encoder_GetSpeed(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
