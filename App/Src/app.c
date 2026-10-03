#include "app.h"

#include "encoder.h"
#include "motor.h"
#include "mpu6050_api.h"
#include "pid.h"
#include "vofa.h"
#include "main.h"

#if APP_CONTROL_PERIOD_MS == 0U
#error "APP_CONTROL_PERIOD_MS must be greater than zero"
#endif

static PID_t angle_pid;
static PID_t speed_pid;
static uint32_t app_last_update;
static VofaMode_t app_last_mode = VOFA_MODE_ANGLE;
volatile VofaMode_t vofa_mode = VOFA_MODE_ANGLE;

/* 初始化两级 PID、编码器、MPU6050 和电机驱动。 */
void App_Init(void)
{
    PID_Init(&angle_pid, APP_ANGLE_KP, APP_ANGLE_KI, APP_ANGLE_KD,
             APP_MAX_TARGET_SPEED, APP_ANGLE_MAX_INTEGRAL);
    PID_Init(&speed_pid, APP_SPEED_KP, APP_SPEED_KI, APP_SPEED_KD,
             APP_MAX_PWM, APP_SPEED_MAX_INTEGRAL);

    Encoder_Init();
    MPU6050_Init();
    Motor_Init();
    app_last_update = HAL_GetTick();
}

/* 按固定周期更新传感器、控制环、电机输出和当前 VOFA 数据帧。 */
void App_Loop(void)
{
    uint32_t now = HAL_GetTick();
    float target_angle;
    float actual_angle;
    float target_speed;
    float actual_speed;
    float output;
    int16_t encoder_count;
    VofaMode_t current_mode;

    if ((uint32_t)(now - app_last_update) < APP_CONTROL_PERIOD_MS)
    {
        return;
    }

    app_last_update = now;
    MPU6050_Update();

    current_mode = vofa_mode;
    if (current_mode != app_last_mode)
    {
        PID_Reset(&angle_pid);
        PID_Reset(&speed_pid);
        app_last_mode = current_mode;
    }

    target_angle = VOFA_GetTarget();
    actual_angle = MPU6050_GetAngle();
    actual_speed = Encoder_GetSpeed();

    if (current_mode == VOFA_MODE_SPEED)
    {
        target_speed = target_angle;
    }
    else
    {
        target_speed = PID_Calc(&angle_pid, target_angle, actual_angle);
    }

    output = PID_Calc(&speed_pid, target_speed, actual_speed);

    Motor_SetPWM((int16_t)output);

    switch (current_mode)
    {
        case VOFA_MODE_SPEED:
            VOFA_SendThree(target_speed, actual_speed, output);
            break;

        case VOFA_MODE_ENCODER:
            encoder_count = Encoder_GetCount();
            VOFA_SendThree((float)encoder_count, 0.0f, 0.0f);
            break;

        case VOFA_MODE_IMU:
        {
            float gyro[3];
            float gyro_accel[3];

            MPU6050_GetGyro(gyro);
            MPU6050_GetGyroAccel(gyro_accel);
            VOFA_SendThree(gyro[0], gyro_accel[0], actual_angle);
            break;
        }

        case VOFA_MODE_ANGLE:
        default:
            VOFA_SendThree(target_angle, actual_angle, output);
            break;
    }
}
