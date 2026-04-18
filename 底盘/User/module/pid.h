#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief 基本 PID 控制器
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float output_limit;
    float last_error;
} pid_t;

/**
 * @brief 初始化 PID 参数和状态
 * @param[out] pid PID 控制器对象
 * @param[in] kp 比例系数
 * @param[in] ki 积分系数
 * @param[in] kd 微分系数
 * @param[in] integral_limit 积分限幅
 * @param[in] output_limit 输出限幅
 * @retval none
 */
void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);

/**
 * @brief 计算一次 PID 输出
 * @param[in,out] pid PID 控制器对象
 * @param[in] ref 目标值
 * @param[in] fdb 反馈值
 * @retval 返回限幅后的控制输出
 */
float pid_calculate(pid_t *pid, float ref, float fdb);

/**
 * @brief 清零 PID 积分项和上一次误差
 * @param[in,out] pid PID 控制器对象
 * @retval none
 */
void pid_reset(pid_t *pid);

#endif
