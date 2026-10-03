#include "mpu6050_api.h"

#include "i2c.h"
#include "ins.h"
#include "main.h"

#ifndef MPU6050_I2C_ADDRESS
#define MPU6050_I2C_ADDRESS (0x68U << 1)
#endif

static uint8_t mpu6050_initialized;

/* 检查 I2C1 设备应答，再初始化驱动和姿态解算。 */
void MPU6050_Init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_I2C_ADDRESS, 3U, 100U) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    ins_init();
    ins_update();
    mpu6050_initialized = 1U;
}

/* 更新一次姿态解算数据。 */
void MPU6050_Update(void)
{
    if (mpu6050_initialized != 0U)
    {
        ins_update();
    }
}

/* 读取最近更新的 roll 角。 */
float MPU6050_GetRoll(void)
{
    return ins_data()->roll;
}

/* 读取最近更新的 pitch 角。 */
float MPU6050_GetPitch(void)
{
    return ins_data()->pitch;
}

/* 读取最近更新的 yaw 角。 */
float MPU6050_GetYaw(void)
{
    return ins_data()->yaw;
}

/* 根据配置选择 roll、pitch 或 yaw 作为控制角。 */
float MPU6050_GetAngle(void)
{
#if MPU6050_CONTROL_ANGLE == MPU6050_ANGLE_ROLL
    return MPU6050_GetRoll();
#elif MPU6050_CONTROL_ANGLE == MPU6050_ANGLE_PITCH
    return MPU6050_GetPitch();
#elif MPU6050_CONTROL_ANGLE == MPU6050_ANGLE_YAW
    return MPU6050_GetYaw();
#else
#error "MPU6050_CONTROL_ANGLE must select ROLL, PITCH, or YAW"
#endif
}

/* 复制最近一帧的三轴角速度到调用者缓冲区。 */
void MPU6050_GetGyro(float gyro[3])
{
    const float *values;
    uint8_t i;

    if (gyro == 0)
    {
        return;
    }

    values = ins_get_gyro();
    for (i = 0U; i < 3U; ++i)
    {
        gyro[i] = values[i];
    }
}

/* 复制最近一帧的三轴角加速度到调用者缓冲区。 */
void MPU6050_GetGyroAccel(float gyro_accel[3])
{
    const float *values;
    uint8_t i;

    if (gyro_accel == 0)
    {
        return;
    }

    values = ins_get_gyro_accel();
    for (i = 0U; i < 3U; ++i)
    {
        gyro_accel[i] = values[i];
    }
}
