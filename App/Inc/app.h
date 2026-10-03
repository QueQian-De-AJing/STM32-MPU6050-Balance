#ifndef APP_H
#define APP_H

/* App 模块：协调传感器、电机、编码器、PID 控制环和 VOFA 数据输出。 */

typedef enum
{
    VOFA_MODE_ANGLE = 0,   /* 目标角度、实际角度、输出 */
    VOFA_MODE_SPEED = 1,   /* 目标速度、实际速度、输出 */
    VOFA_MODE_ENCODER = 2, /* 编码器计数 */
    VOFA_MODE_IMU = 3      /* X 轴角速度、X 轴角加速度、姿态角 */
} VofaMode_t;

/* 控制周期及初始 PID 参数，可按实际电机、编码器和机构调整。 */
#ifndef APP_CONTROL_PERIOD_MS
#define APP_CONTROL_PERIOD_MS 10U
#endif

#ifndef APP_ANGLE_KP
#define APP_ANGLE_KP 8.0f
#endif

#ifndef APP_ANGLE_KI
#define APP_ANGLE_KI 0.0f
#endif

#ifndef APP_ANGLE_KD
#define APP_ANGLE_KD 0.0f
#endif

#ifndef APP_MAX_TARGET_SPEED
#define APP_MAX_TARGET_SPEED 500.0f
#endif

#ifndef APP_ANGLE_MAX_INTEGRAL
#define APP_ANGLE_MAX_INTEGRAL 500.0f
#endif

#ifndef APP_SPEED_KP
#define APP_SPEED_KP 0.5f
#endif

#ifndef APP_SPEED_KI
#define APP_SPEED_KI 0.02f
#endif

#ifndef APP_SPEED_KD
#define APP_SPEED_KD 0.0f
#endif

#ifndef APP_MAX_PWM
#define APP_MAX_PWM 800.0f
#endif

#ifndef APP_SPEED_MAX_INTEGRAL
#define APP_SPEED_MAX_INTEGRAL 8000.0f
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 当前 VOFA 显示及控制模式；VOFA_MODE_SPEED 下接收目标直接作为速度。 */
extern volatile VofaMode_t vofa_mode;

/* 初始化 MPU6050、编码器、电机和角度/速度两级 PID。 */
void App_Init(void);

/* 按固定周期更新姿态和串级 PID，并通过 VOFA 发送目标角、实际角和 PWM。 */
void App_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
