#include "encoder.h"

static uint16_t encoder_last_count;

/* 清零计数器和软件基准，并启动定时器编码器接口。 */
void Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(ENCODER_TIMER, 0U);
    encoder_last_count = 0U;

    if (HAL_TIM_Encoder_Start(ENCODER_TIMER, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
        return;
    }
}

/* 读取当前定时器计数并转换为有符号 16 位值。 */
int16_t Encoder_GetCount(void)
{
    int32_t count = (uint16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIMER);

    if (count > INT16_MAX)
    {
        count -= 65536;
    }

    return (int16_t)count;
}

/* 读取相对上次采样的计数差，并修正 16 位计数回绕。 */
int16_t Encoder_GetDelta(void)
{
    uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIMER);
    int32_t delta = (int32_t)count - (int32_t)encoder_last_count;

    /* 将差值映射到 16 位有符号范围，修正 CNT 正反向回绕。 */
    if (delta > INT16_MAX)
    {
        delta -= 65536;
    }
    else if (delta < INT16_MIN)
    {
        delta += 65536;
    }

    encoder_last_count = count;
    return (int16_t)delta;
}

/* 按配置的采样周期将计数差换算为每秒计数数。 */
float Encoder_GetSpeed(void)
{
    return (float)Encoder_GetDelta() * 1000.0f / (float)ENCODER_SAMPLE_TIME_MS;
}
