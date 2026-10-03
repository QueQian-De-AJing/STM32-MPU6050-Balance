#include "vofa.h"

#include <stdlib.h>
#include "usart.h"

#ifndef VOFA_UART_HANDLE
#define VOFA_UART_HANDLE (&huart1)
#endif

#define VOFA_RX_LINE_SIZE 32U
#define VOFA_TX_BUFFER_SIZE 64U
#define VOFA_DECIMAL_DIGITS 3U
#define VOFA_MAX_FORMAT_VALUE 4000000000.0f

static char vofa_rx_line[VOFA_RX_LINE_SIZE];
static uint8_t vofa_rx_length;
static uint8_t vofa_rx_discard_line;
static volatile float vofa_target;

static uint16_t VOFA_AppendUnsigned(char *buffer, uint32_t value)
{
    char digits[10];
    uint16_t count = 0U;
    uint16_t index;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    for (index = 0U; index < count; ++index)
    {
        buffer[index] = digits[count - index - 1U];
    }

    return count;
}

static uint16_t VOFA_AppendFloat(char *buffer, float value)
{
    uint16_t length = 0U;
    uint32_t whole;
    uint32_t fraction;
    float magnitude;
    uint8_t digit;

    if (value != value)
    {
        buffer[0] = 'n';
        buffer[1] = 'a';
        buffer[2] = 'n';
        return 3U;
    }

    if (value < 0.0f)
    {
        buffer[length++] = '-';
        magnitude = -value;
    }
    else
    {
        magnitude = value;
    }

    if (magnitude > VOFA_MAX_FORMAT_VALUE)
    {
        magnitude = VOFA_MAX_FORMAT_VALUE;
    }

    whole = (uint32_t)magnitude;
    fraction = (uint32_t)(((magnitude - (float)whole) * 1000.0f) + 0.5f);
    if (fraction >= 1000U)
    {
        ++whole;
        fraction = 0U;
    }

    length += VOFA_AppendUnsigned(&buffer[length], whole);
    buffer[length++] = '.';
    for (digit = 0U; digit < VOFA_DECIMAL_DIGITS; ++digit)
    {
        uint32_t divisor = (digit == 0U) ? 100U : ((digit == 1U) ? 10U : 1U);
        buffer[length++] = (char)('0' + ((fraction / divisor) % 10U));
    }

    return length;
}

/* 清空接收行状态和目标值，外设初始化由 USART 模块负责。 */
void VOFA_Init(void)
{
    vofa_rx_length = 0U;
    vofa_rx_discard_line = 0U;
    vofa_target = 0.0f;
}

/* 格式化三个浮点数并通过配置的 UART 发送 FireWater 文本帧。 */
void VOFA_SendFloat(float a, float b, float c)
{
    char buffer[VOFA_TX_BUFFER_SIZE];
    uint16_t length = 0U;

    length += VOFA_AppendFloat(&buffer[length], a);
    buffer[length++] = ',';
    length += VOFA_AppendFloat(&buffer[length], b);
    buffer[length++] = ',';
    length += VOFA_AppendFloat(&buffer[length], c);
    buffer[length++] = '\n';

    (void)HAL_UART_Transmit(VOFA_UART_HANDLE, (uint8_t *)buffer, length, HAL_MAX_DELAY);
}

/* 按目标值、实际值、输出的顺序发送三个控制量。 */
void VOFA_SendThree(float target, float actual, float output)
{
    VOFA_SendFloat(target, actual, output);
}

/* 累积接收字节；遇到换行后解析并更新目标值。 */
void VOFA_ParseByte(uint8_t byte)
{
    char *end;
    float parsed;

    if (byte == '\n')
    {
        if ((vofa_rx_discard_line == 0U) && (vofa_rx_length > 0U))
        {
            vofa_rx_line[vofa_rx_length] = '\0';
            parsed = strtof(vofa_rx_line, &end);
            if (end != vofa_rx_line)
            {
                vofa_target = parsed;
            }
        }

        vofa_rx_length = 0U;
        vofa_rx_discard_line = 0U;
        return;
    }

    if (vofa_rx_discard_line != 0U)
    {
        return;
    }

    if (vofa_rx_length < (VOFA_RX_LINE_SIZE - 1U))
    {
        vofa_rx_line[vofa_rx_length++] = (char)byte;
    }
    else
    {
        vofa_rx_length = 0U;
        vofa_rx_discard_line = 1U;
    }
}

/* 返回最近一次成功解析的目标值。 */
float VOFA_GetTarget(void)
{
    return vofa_target;
}
