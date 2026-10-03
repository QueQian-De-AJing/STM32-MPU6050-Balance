/* MPU6050 驱动模块：通过 I2C1 读写传感器并输出校准后的原始物理量。 */

#ifndef THIRD_PARTY_MPU6050_H
#define THIRD_PARTY_MPU6050_H

#include <stdint.h>

/** MPU6050 数据（已换算为物理量） */
typedef struct
{
    float accel[3]; /* 加速度，单位 g */
    float gyro[3];  /* 角速度，单位 °/s（已去除零偏） */
    float temp;     /* 温度，单位 °C */
} bsp_mpu6050_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 MPU6050，量程默认 ±2g / ±250°/s，数字低通 44 Hz。
 */
void bsp_mpu6050_init(void);

/**
 * 读取 MPU6050 数据，返回换算后的加速度、角速度与温度。
 */
bsp_mpu6050_data_t bsp_mpu6050_read(void);

/**
 * 校准陀螺仪零偏，调用期间请保持静止。
 * @param samples 校准采样次数。
 */
void bsp_mpu6050_calibrate(uint16_t samples);

#ifdef __cplusplus
}
#endif

#endif /* THIRD_PARTY_MPU6050_H */
