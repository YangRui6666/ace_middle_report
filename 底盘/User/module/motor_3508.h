#ifndef MOTOR_3508_H
#define MOTOR_3508_H

#include <stdint.h>

/**
 * @brief 单个 M3508 电机控制状态
 */
typedef struct
{
    int16_t target_rpm;
    int16_t feedback_rpm;
    int16_t output_current;
} motor_3508_state_t;

/**
 * @brief 初始化四个 M3508 电机速度环
 * @param[in] none
 * @retval none
 */
void motor_3508_init(void);

/**
 * @brief 清零四个电机的 PID 历史状态和输出状态
 * @param[in] none
 * @retval none
 */
void motor_3508_reset(void);

/**
 * @brief 设置四个电机的目标转速
 * @param[in] target_rpm 四轮目标转速
 * @retval none
 */
void motor_3508_set_target(const int16_t target_rpm[4]);

/**
 * @brief 执行一次四电机速度环计算并发送电流
 * @param[in] none
 * @retval none
 */
void motor_3508_control_step(void);

/**
 * @brief 停止四个电机输出并持续发送零电流
 * @param[in] none
 * @retval none
 */
void motor_3508_stop_output(void);

/**
 * @brief 获取四个底盘电机的最新反馈转速
 * @param[out] feedback_rpm 四轮反馈转速缓存
 * @retval none
 */
void motor_3508_get_feedback_rpm(int16_t feedback_rpm[4]);

/**
 * @brief 获取指定电机控制状态
 * @param[in] index 电机序号，范围为 1~4
 * @param[out] state 电机状态输出缓存
 * @retval none
 */
void motor_3508_get_state(uint8_t index, motor_3508_state_t *state);

#endif
