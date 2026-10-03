#ifndef MOTOR_H
#define MOTOR_H

/* Motor 模块：通过 TB6612 控制电机方向与 PWM 调速。 */

#include "main.h"
#include "tim.h"

/* TB6612 引脚和 PWM 配置占位宏，可在此处或编译选项中统一替换。 */
#ifndef MOTOR_AIN1_GPIO_PORT
#define MOTOR_AIN1_GPIO_PORT AIN1_GPIO_Port
#endif

#ifndef MOTOR_AIN1_PIN
#define MOTOR_AIN1_PIN AIN1_Pin
#endif

#ifndef MOTOR_AIN2_GPIO_PORT
#define MOTOR_AIN2_GPIO_PORT AIN2_GPIO_Port
#endif

#ifndef MOTOR_AIN2_PIN
#define MOTOR_AIN2_PIN AIN2_Pin
#endif

#ifndef MOTOR_STBY_GPIO_PORT
#define MOTOR_STBY_GPIO_PORT STBY_GPIO_Port
#endif

#ifndef MOTOR_STBY_PIN
#define MOTOR_STBY_PIN STBY_Pin
#endif

#ifndef MOTOR_PWMA_TIMER
#define MOTOR_PWMA_TIMER (&htim2)
#endif

#ifndef MOTOR_PWMA_CHANNEL
#define MOTOR_PWMA_CHANNEL TIM_CHANNEL_1
#endif

#ifndef MOTOR_PWM_MIN
#define MOTOR_PWM_MIN (-1000)
#endif

#ifndef MOTOR_PWM_MAX
#define MOTOR_PWM_MAX 1000
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 在 CubeMX 的 GPIO、PWM 定时器初始化后调用；以停止状态启动 PWM 并使能 TB6612。 */
void Motor_Init(void);

/* 设置电机 PWM：正数正转，负数反转，0 停止；输入范围限幅到 -1000~1000。 */
void Motor_SetPWM(int16_t pwm);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
