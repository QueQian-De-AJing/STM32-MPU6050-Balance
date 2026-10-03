//
// INS 姿态解算（Mahony 互补滤波）
//

#include "ins.h"

#include <math.h>
#include <string.h>

#include "main.h"

// 滤波参数
#define INS_KP           0.8f           // 加速度计比例增益（越大收敛越快、越受振动影响）
#define INS_GYRO_ACC_LPF 0.3f           // 角加速度低通系数（0~1）
#define RAD2DEG          57.2957795f    // 弧度转角度
#define DEG2RAD          0.0174532925f  // 角度转弧度

static ins_data_t ins_state;

static float previous_gyro[3];       // 上一次的角速度，用于求角加速度
static uint32_t last_update_tick;     // 上一次更新的时间戳
static uint8_t ins_initialized;       // 是否已初始化
static uint8_t first_update;          // 是否为第一次更新
static int16_t yaw_rotation_count;    // 偏航角累计圈数
static float previous_yaw_angle;      // 上一次的偏航角，用于判断跳变

/* 初始化滤波状态，并在传感器静止时完成陀螺仪零偏校准。 */
void ins_init(void)
{
    bsp_mpu6050_init();
    bsp_mpu6050_calibrate(500);  // 上电静止校准陀螺零偏

    memset(&ins_state, 0, sizeof(ins_state));
    ins_state.q[0] = 1.0f;  // 初始姿态为单位四元数

    memset(previous_gyro, 0, sizeof(previous_gyro));
    yaw_rotation_count = 0;
    previous_yaw_angle = 0.0f;
    first_update = 1;
    last_update_tick = HAL_GetTick();
    ins_initialized = 1;
}

/* 读取一帧传感器数据并更新滤波、姿态角和角加速度。 */
void ins_update(void)
{
    if (!ins_initialized)
    {
        return;
    }

    // 读取底层数据并更新时间戳
    bsp_mpu6050_data_t raw = bsp_mpu6050_read();
    ins_state.raw = raw;
    ins_state.timestamp = HAL_GetTick();

    // 计算积分周期 dt
    float dt = (float)(ins_state.timestamp - last_update_tick) / 1000.0f;
    last_update_tick = ins_state.timestamp;
    if (dt <= 0.0f || dt > 0.5f)
    {
        dt = 0.001f;
    }

    // 角速度直接输出（°/s）
    for (uint8_t i = 0; i < 3; i++)
    {
        ins_state.gyro[i] = raw.gyro[i];
    }

    // 角加速度 = 角速度差分，一阶低通抑制噪声（°/s²）
    if (first_update)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            previous_gyro[i] = raw.gyro[i];
            ins_state.gyro_accel[i] = 0.0f;
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            float da = (raw.gyro[i] - previous_gyro[i]) / dt;
            ins_state.gyro_accel[i] += INS_GYRO_ACC_LPF * (da - ins_state.gyro_accel[i]);
            previous_gyro[i] = raw.gyro[i];
        }
    }

    // 加速度计归一化，得到重力方向
    float ax = raw.accel[0], ay = raw.accel[1], az = raw.accel[2];
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm > 0.0001f)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
    }

    float q0 = ins_state.q[0], q1 = ins_state.q[1], q2 = ins_state.q[2], q3 = ins_state.q[3];

    // 由四元数估计重力方向（半矢量）
    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    // 测量重力与估计重力的叉积，作为姿态误差
    float halfex = ay * halfvz - az * halfvy;
    float halfey = az * halfvx - ax * halfvz;
    float halfez = ax * halfvy - ay * halfvx;

    // 比例反馈修正角速度（四元数积分需要 rad/s）
    float gx = raw.gyro[0] * DEG2RAD + 2.0f * INS_KP * halfex;
    float gy = raw.gyro[1] * DEG2RAD + 2.0f * INS_KP * halfey;
    float gz = raw.gyro[2] * DEG2RAD + 2.0f * INS_KP * halfez;

    // 一阶龙格库塔法积分四元数
    float half_t = 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_t;
    q1 += ( qa * gx + qc * gz - q3 * gy) * half_t;
    q2 += ( qa * gy - qb * gz + q3 * gx) * half_t;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_t;

    // 四元数归一化
    float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_norm > 0.0001f)
    {
        q0 /= q_norm;
        q1 /= q_norm;
        q2 /= q_norm;
        q3 /= q_norm;
    }
    ins_state.q[0] = q0;
    ins_state.q[1] = q1;
    ins_state.q[2] = q2;
    ins_state.q[3] = q3;

    // 四元数转欧拉角（输出为 °）
    ins_state.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    float sp = 2.0f * (q0 * q2 - q3 * q1);
    if (sp > 1.0f)
    {
        sp = 1.0f;
    }
    else if (sp < -1.0f)
    {
        sp = -1.0f;
    }
    ins_state.pitch = asinf(sp) * RAD2DEG;
    ins_state.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;

    // 累计偏航角，处理 ±180° 跳变
    if (first_update)
    {
        previous_yaw_angle = ins_state.yaw;
        first_update = 0;
    }
    float d_yaw = ins_state.yaw - previous_yaw_angle;
    if (d_yaw < -180.0f)
    {
        yaw_rotation_count++;
    }
    else if (d_yaw > 180.0f)
    {
        yaw_rotation_count--;
    }
    previous_yaw_angle = ins_state.yaw;
    ins_state.yaw_total_angle = ins_state.yaw + (float)yaw_rotation_count * 360.0f;
}

/* 返回内部姿态数据结构的指针。 */
ins_data_t *ins_data(void)
{
    return &ins_state;
}

/* 返回最近一帧的三轴角速度数组。 */
const float *ins_get_gyro(void)
{
    return ins_state.gyro;
}

/* 返回最近一帧的三轴角加速度数组。 */
const float *ins_get_gyro_accel(void)
{
    return ins_state.gyro_accel;
}
