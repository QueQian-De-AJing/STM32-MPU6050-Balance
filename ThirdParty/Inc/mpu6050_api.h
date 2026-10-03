#ifndef MPU6050_API_H
#define MPU6050_API_H

/* MPU6050 适配模块：统一提供姿态角、角速度和角加速度读取接口。 */

#include <stdint.h>

#define MPU6050_ANGLE_ROLL  0U
#define MPU6050_ANGLE_PITCH 1U
#define MPU6050_ANGLE_YAW   2U

/* 传感器安装方向确认后，可改为 MPU6050_ANGLE_PITCH 或 MPU6050_ANGLE_YAW。 */
#ifndef MPU6050_CONTROL_ANGLE
#define MPU6050_CONTROL_ANGLE MPU6050_ANGLE_ROLL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 I2C1 上的 MPU6050 和姿态解算；陀螺仪校准期间传感器须保持静止。 */
void MPU6050_Init(void);

/* 周期调用一次，读取传感器并更新姿态解算结果，建议每 10 ms 调用。 */
void MPU6050_Update(void);

/* 返回配置为控制轴的姿态角；当前默认 roll，安装方向确认后可切换宏。 */
float MPU6050_GetAngle(void);

/* 读取最近一次更新的 roll 姿态角，单位为度。 */
float MPU6050_GetRoll(void);

/* 读取最近一次更新的 pitch 姿态角，单位为度。 */
float MPU6050_GetPitch(void);

/* 读取最近一次更新的 yaw 姿态角，单位为度；MPU6050 无磁力计，yaw 会漂移。 */
float MPU6050_GetYaw(void);

/* 读取最近一次更新的陀螺仪角速度，单位为度/秒。 */
void MPU6050_GetGyro(float gyro[3]);

/* 读取最近一次更新的陀螺仪角加速度，单位为度/秒^2。 */
void MPU6050_GetGyroAccel(float gyro_accel[3]);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_API_H */
