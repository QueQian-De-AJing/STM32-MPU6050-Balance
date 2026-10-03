#ifndef VOFA_H
#define VOFA_H

/* VOFA 模块：通过 FireWater 文本帧收发浮点数据并解析目标值。 */

#include <stdint.h>

/*
 * VOFA+ FireWater 简单文本格式：float1,float2,float3\n。
 * 上位机选择串口和 FireWater/文本接收，发送端按逗号分列；
 * 下行发送一个目标值并以换行结尾。串口接收完成回调中调用
 * VOFA_ParseByte(byte)，并重新启动 HAL_UART_Receive_IT() 接收。
 */

/* 初始化 USART 发送模块和接收行解析状态；USART 外设由 CubeMX 初始化。 */
void VOFA_Init(void);

/* 通过 USART1 发送三个逗号分隔的浮点数，末尾为换行符。 */
void VOFA_SendFloat(float a, float b, float c);

/* 通过 FireWater 发送目标值、实际值和控制输出。 */
void VOFA_SendThree(float target, float actual, float output);

/* 在串口接收中断回调中逐字节调用；遇到换行时解析并保存目标值。 */
void VOFA_ParseByte(uint8_t byte);

/* 获取最近一行解析到的第一个浮点数；无有效行时返回初始值 0。 */
float VOFA_GetTarget(void);

#endif /* VOFA_H */
