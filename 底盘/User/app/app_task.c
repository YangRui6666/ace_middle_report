#include "app_task.h"

#include <stdint.h>

#include "cmsis_os.h"

#include "app_config.h"
#include "bsp_can.h"
#include "bsp_dbus.h"
#include "chassis.h"
#include "imu_fusion.h"
#include "motor_3508.h"

static uint8_t ctrl_inited;
static uint8_t motor_inited;

/**
 * @brief 初始化控制任务依赖模块
 * @param[in] none
 * @retval none
 * @attention
 * 该函数只执行一次，负责接通遥控输入和底盘目标计算链路。
 */
static void app_ctrl_init(void)
{
    if (ctrl_inited != 0U)
    {
        return;
    }

    bsp_dbus_init();
    imu_init();
    chassis_init();
    ctrl_inited = 1U;
}

/**
 * @brief 初始化电机任务依赖模块
 * @param[in] none
 * @retval none
 * @attention
 * 该函数只执行一次，负责接通 CAN 接收发送和速度环控制链路。
 */
static void app_motor_init(void)
{
    if (motor_inited != 0U)
    {
        return;
    }

    bsp_can_init();
    motor_3508_init();
    motor_inited = 1U;
}

/**
 * @brief 控制任务
 * @param[in] argument 任务参数，当前未使用
 * @retval none
 * @attention
 * 周期读取 IMU、遥控器和轮速反馈，计算参考系下的底盘目标速度。
 */
void ctrlStartTask(void *argument)
{
    dbus_data_t remote;
    imu_data_t imu_data;
    int16_t wheel_feedback_rpm[4];
    uint32_t last_wake;

    (void)argument;
    app_ctrl_init();
    last_wake = osKernelGetTickCount();

    while (1)
    {
        imu_update();
        imu_get_data(&imu_data);
        bsp_dbus_get_data(&remote);
        motor_3508_get_feedback_rpm(wheel_feedback_rpm);
        chassis_control_step(&remote, &imu_data, wheel_feedback_rpm);

        osDelayUntil(last_wake + CTRL_TASK_PERIOD_MS);
        last_wake += CTRL_TASK_PERIOD_MS;
    }
}

/**
 * @brief 电机任务
 * @param[in] argument 任务参数，当前未使用
 * @retval none
 * @attention
 * 周期读取底盘给出的四轮目标转速，执行速度环并发送电流。
 */
void motorStartTask(void *argument)
{
    int16_t wheel_rpm[4];
    uint32_t last_wake;

    (void)argument;
    app_motor_init();
    last_wake = osKernelGetTickCount();

    while (1)
    {
        if (chassis_is_output_enabled() != 0U)
        {
            chassis_get_wheel_rpm(wheel_rpm);
            motor_3508_set_target(wheel_rpm);
            motor_3508_control_step();
        }
        else
        {
            motor_3508_stop_output();
        }

        osDelayUntil(last_wake + MOTOR_TASK_PERIOD_MS);
        last_wake += MOTOR_TASK_PERIOD_MS;
    }
}
