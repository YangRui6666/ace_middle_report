#include "motor_3508.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_config.h"
#include "bsp_can.h"
#include "pid.h"
#include "task.h"

typedef struct
{
    pid_t speed_pid;
    motor_3508_state_t state;
} motor_3508_controller_t;

static motor_3508_controller_t motors[4];

/**
 * @brief 初始化四个 M3508 速度环
 * @param[in] none
 * @retval none
 */
void motor_3508_init(void)
{
    uint8_t i;

    memset(motors, 0, sizeof(motors));
    for (i = 0; i < 4U; i++)
    {
        pid_init(&motors[i].speed_pid,
                 M3508_SPEED_KP,
                 M3508_SPEED_KI,
                 M3508_SPEED_KD,
                 M3508_INTEGRAL_LIMIT,
                 M3508_CURRENT_LIMIT);
    }
}

/**
 * @brief 清零四个电机的 PID 历史状态和输出状态
 * @param[in] none
 * @retval none
 */
void motor_3508_reset(void)
{
    uint8_t i;

    taskENTER_CRITICAL();
    for (i = 0; i < 4U; i++)
    {
        pid_reset(&motors[i].speed_pid);
        motors[i].state.target_rpm = 0;
        motors[i].state.feedback_rpm = 0;
        motors[i].state.output_current = 0;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 设置四个电机的目标转速
 * @param[in] target_rpm 四轮目标转速
 * @retval none
 */
void motor_3508_set_target(const int16_t target_rpm[4])
{
    uint8_t i;

    taskENTER_CRITICAL();
    for (i = 0; i < 4U; i++)
    {
        motors[i].state.target_rpm = target_rpm[i];
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 执行一次速度环控制
 * @param[in] none
 * @retval none
 * @attention
 * 该函数会先读取 CAN 反馈，再计算四个电机输出电流，最后统一发包。
 */
void motor_3508_control_step(void)
{
    can_motor_feedback_t feedback;
    int16_t iq[4];
    uint8_t i;

    for (i = 0; i < 4U; i++)
    {
        bsp_can_get_motor_feedback(i + 1U, &feedback);
        motors[i].state.feedback_rpm = feedback.speed_rpm;
        motors[i].state.output_current = (int16_t)pid_calculate(&motors[i].speed_pid,
                                                                 motors[i].state.target_rpm,
                                                                 motors[i].state.feedback_rpm);
        iq[i] = motors[i].state.output_current;
    }

    bsp_can_set_chassis_current(iq[0], iq[1], iq[2], iq[3]);
}

/**
 * @brief 停止四个电机输出并持续发送零电流
 * @param[in] none
 * @retval none
 */
void motor_3508_stop_output(void)
{
    uint8_t i;

    taskENTER_CRITICAL();
    for (i = 0; i < 4U; i++)
    {
        pid_reset(&motors[i].speed_pid);
        motors[i].state.target_rpm = 0;
        motors[i].state.output_current = 0;
    }
    taskEXIT_CRITICAL();

    bsp_can_set_chassis_current(0, 0, 0, 0);
}

/**
 * @brief 获取四个底盘电机的最新反馈转速
 * @param[out] feedback_rpm 四轮反馈转速缓存
 * @retval none
 */
void motor_3508_get_feedback_rpm(int16_t feedback_rpm[4])
{
    can_motor_feedback_t feedback;
    uint8_t i;

    if (feedback_rpm == NULL)
    {
        return;
    }

    for (i = 0; i < 4U; i++)
    {
        bsp_can_get_motor_feedback(i + 1U, &feedback);
        feedback_rpm[i] = feedback.speed_rpm;
    }
}

/**
 * @brief 获取指定电机的控制状态
 * @param[in] index 电机序号，范围为 1~4
 * @param[out] state 电机状态输出缓存
 * @retval none
 */
void motor_3508_get_state(uint8_t index, motor_3508_state_t *state)
{
    if ((index < 1U) || (index > 4U))
    {
        memset(state, 0, sizeof(*state));
        return;
    }

    taskENTER_CRITICAL();
    *state = motors[index - 1U].state;
    taskEXIT_CRITICAL();
}
