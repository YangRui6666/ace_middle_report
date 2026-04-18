#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

#include "bsp_dbus.h"
#include "imu_fusion.h"

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_NORMAL,
    CHASSIS_MODE_GYROSCOPE,
} chassis_mode_t;

/**
 * @brief 底盘当前状态
 * @attention
 * vx_mm_s、vy_mm_s、wz_rad_s 为最终下发到底盘坐标系的目标速度。
 * vx_cmd_head_mm_s、vy_cmd_head_mm_s 为头部参考系下的操作手目标速度。
 * vx_cmd_world_mm_s、vy_cmd_world_mm_s 为惯性参考系下的平移目标速度。
 * vx_ref_world_mm_s、vy_ref_world_mm_s 为加入速度修正后的惯性系参考速度。
 * vx_meas_body_mm_s、vy_meas_body_mm_s 为由轮速估计得到的车体系速度。
 * vx_meas_world_mm_s、vy_meas_world_mm_s 为旋转到惯性系后的估计速度。
 * head_yaw_world_rad 为虚拟云台角，body_yaw_world_rad 为车体角。
 */
typedef struct
{
    float vx_mm_s;
    float vy_mm_s;
    float wz_rad_s;
    float vx_cmd_head_mm_s;
    float vy_cmd_head_mm_s;
    float vx_cmd_world_mm_s;
    float vy_cmd_world_mm_s;
    float vx_ref_world_mm_s;
    float vy_ref_world_mm_s;
    float vx_meas_body_mm_s;
    float vy_meas_body_mm_s;
    float vx_meas_world_mm_s;
    float vy_meas_world_mm_s;
    float wz_meas_rad_s;
    float head_yaw_world_rad;
    float body_yaw_world_rad;
    float body_yaw_target_rad;
    chassis_mode_t mode;
    int16_t wheel_rpm[4];
    float chassis_max_limit_speed_mm_s;
    uint8_t output_enabled;
} chassis_state_t;

/**
 * @brief 初始化底盘状态
 * @param[in] none
 * @retval none
 */
void chassis_init(void);

/**
 * @brief 清零底盘运动输出，并将参考头向对齐到当前车体朝向
 * @param[in] none
 * @retval none
 */
void chassis_reset(void);

/**
 * @brief 执行一次底盘参考系控制
 * @param[in] remote 遥控器数据
 * @param[in] imu IMU 姿态与角速度数据
 * @param[in] wheel_feedback_rpm 四轮反馈转速
 * @retval none
 */
void chassis_control_step(const dbus_data_t *remote,
                          const imu_data_t *imu,
                          const int16_t wheel_feedback_rpm[4]);

/**
 * @brief 获取四个轮子的目标转速
 * @param[out] wheel_rpm 四轮目标转速缓存
 * @retval none
 */
void chassis_get_wheel_rpm(int16_t wheel_rpm[4]);

/**
 * @brief 获取当前底盘输出是否使能
 * @param[in] none
 * @retval 1 表示允许电机输出，0 表示电机保持零电流
 */
uint8_t chassis_is_output_enabled(void);

/**
 * @brief 获取当前底盘状态快照
 * @param[out] state 底盘状态输出缓存
 * @retval none
 */
void chassis_get_state(chassis_state_t *state);

#endif
