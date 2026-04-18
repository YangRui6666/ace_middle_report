#include "pid.h"

/**
 * @brief 对 PID 中间量做对称限幅
 * @param[in] value 输入值
 * @param[in] limit 限幅值
 * @retval 返回限幅后的结果
 */
static float pid_limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

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
void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->last_error = 0.0f;
}

/**
 * @brief 计算一次 PID 输出
 * @param[in,out] pid PID 控制器对象
 * @param[in] ref 目标值
 * @param[in] fdb 反馈值
 * @retval 返回限幅后的控制输出
 */
float pid_calculate(pid_t *pid, float ref, float fdb)
{
    float error;
    float derivative;
    float output;

    error = ref - fdb;
    pid->integral += error;
    pid->integral = pid_limit(pid->integral, pid->integral_limit);

    derivative = error - pid->last_error;
    pid->last_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return pid_limit(output, pid->output_limit);
}

/**
 * @brief 清零 PID 历史状态
 * @param[in,out] pid PID 控制器对象
 * @retval none
 */
void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}
