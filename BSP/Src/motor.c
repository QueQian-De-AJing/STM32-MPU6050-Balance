#include "motor.h"

/* 初始化 TB6612 控制脚，并以零占空比启动 PWM。 */
void Motor_Init(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(MOTOR_PWMA_TIMER, MOTOR_PWMA_CHANNEL, 0U);
    if (HAL_TIM_PWM_Start(MOTOR_PWMA_TIMER, MOTOR_PWMA_CHANNEL) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

/* 设置转向和 PWM 占空比，输入先限制在配置范围内。 */
void Motor_SetPWM(int16_t pwm)
{
    int32_t duty;

    if (pwm > MOTOR_PWM_MAX)
    {
        pwm = MOTOR_PWM_MAX;
    }
    else if (pwm < MOTOR_PWM_MIN)
    {
        pwm = MOTOR_PWM_MIN;
    }

    if (pwm > 0)
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    }
    else if (pwm < 0)
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    }

    duty = (pwm < 0) ? -(int32_t)pwm : (int32_t)pwm;
    __HAL_TIM_SET_COMPARE(MOTOR_PWMA_TIMER, MOTOR_PWMA_CHANNEL, (uint32_t)duty);
}
