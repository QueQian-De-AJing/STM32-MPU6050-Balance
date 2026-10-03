/* INS 模块：融合 MPU6050 六轴数据，计算姿态角、角速度和角加速度。 */

#ifndef THIRD_PARTY_INS_H
#define THIRD_PARTY_INS_H

#include <stdint.h>

#include "mpu6050.h"

/** 姿态解算输出 */
typedef struct
{
    float q[4];             /* 姿态四元数 [w, x, y, z] */
    float roll, pitch, yaw; /* 姿态角，单位 ° */
    float yaw_total_angle;  /* 累计偏航角，单位 °，可超过 ±180 */
    float gyro[3];          /* 角速度，单位 °/s */
    float gyro_accel[3];    /* 角加速度，单位 °/s² */
    uint32_t timestamp;     /* 更新时间戳，单位 ms */
    bsp_mpu6050_data_t raw; /* 本次读取的底层数据 */
} ins_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化姿态解算并校准陀螺仪零偏，期间请保持传感器静止。
 */
void ins_init(void);

/**
 * 读取一次传感器数据并更新姿态解算结果，需周期性调用。
 */
void ins_update(void);

/**
 * 获取姿态解算结果结构体。
 */
ins_data_t *ins_data(void);

/**
 * 获取角速度数组（°/s），数组长度为 3。
 */
const float *ins_get_gyro(void);

/**
 * 获取角加速度数组（°/s²），数组长度为 3。
 */
const float *ins_get_gyro_accel(void);

#ifdef __cplusplus
}
#endif

#endif /* THIRD_PARTY_INS_H */
