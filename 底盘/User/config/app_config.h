#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/**
 * @file app_config.h
 * @brief 工程级全局配置宏，集中管理任务周期、遥控器参数、底盘参数和电机参数。
 */

/* FreeRTOS 任务周期配置，单位：ms */
/* 控制任务调度周期 */
#define CTRL_TASK_PERIOD_MS            5U
/* 电机任务调度周期 */
#define MOTOR_TASK_PERIOD_MS           2U

/* DT7/DBUS 接收配置 */
/* DT7 遥控器完整数据帧长度，单位：字节 */
#define DBUS_FRAME_SIZE                18U
/* 摇杆通道原始中值，解码后以 0 为中心 */
#define DBUS_CHANNEL_CENTER            1024
/* 摇杆通道原始最小值 */
#define DBUS_CHANNEL_MIN               364
/* 摇杆通道原始最大值 */
#define DBUS_CHANNEL_MAX               1684
/* 摇杆通道死区，单位：原始码值 */
#define DBUS_CHANNEL_DEADBAND          10
/* 遥控器失联判定超时时间，单位：ms */
#define DBUS_LOSS_TIMEOUT_MS           100U

/* 底盘机械参数 */
/* 底盘轮半径，单位：mm */
#define CHASSIS_WHEEL_RADIUS_MM        81.5f
/* 底盘对角轮中心距，单位：mm */
#define CHASSIS_WHEEL_DIAGONAL_MM      425.0f
/* 3508 电机减速比分子 */
#define CHASSIS_GEAR_RATIO_NUM         268.0f
/* 3508 电机减速比分母 */
#define CHASSIS_GEAR_RATIO_DEN         17.0f
/* 3508 电机总减速比 */
#define CHASSIS_GEAR_RATIO             (CHASSIS_GEAR_RATIO_NUM / CHASSIS_GEAR_RATIO_DEN)
/* 底盘轮周长，单位：mm */
#define CHASSIS_WHEEL_CIRCUM_MM        (2.0f * 3.1415926f * CHASSIS_WHEEL_RADIUS_MM)
/* 底盘旋转半径，单位：mm */
#define CHASSIS_ROTATE_RADIUS_MM       (CHASSIS_WHEEL_DIAGONAL_MM * 0.5f)

/* 底盘速度上限 */
/* 底盘最大平移速度，单位：mm/s */
#define CHASSIS_MAX_LINEAR_SPEED_MM_S  2000.0f
/* 底盘最大角速度，单位：rad/s */
#define CHASSIS_MAX_ANGULAR_SPEED_RAD  3.5f
/* 头向最大偏航速率，单位：rad/s */
#define CHASSIS_MAX_HEAD_YAW_RATE_RAD_S 3.5f
/* 陀螺仪模式下的旋转速度，取最大角速度的 80% */
#define CHASSIS_GYROSCOPE_SPIN_SPEED_RAD_S (CHASSIS_MAX_ANGULAR_SPEED_RAD * 0.8f)
/* S2 中档速度倍率 */
#define CHASSIS_S2_SPEED_RATIO_MID     1.0f
/* S2 下档速度倍率 */
#define CHASSIS_S2_SPEED_RATIO_DOWN    2.0f

/* 底盘参考系控制参数 */
/* 车体朝向保持比例系数 */
#define CHASSIS_YAW_HOLD_KP            4.0f
/* 车体朝向保持微分系数 */
#define CHASSIS_YAW_HOLD_KD            0.12f
/* 世界系速度 PI 比例系数 */
#define CHASSIS_WORLD_SPEED_KP         0.25f
/* 世界系速度 PI 积分系数 */
#define CHASSIS_WORLD_SPEED_KI         0.8f
/* 世界系速度积分限幅，单位：mm/s */
#define CHASSIS_WORLD_SPEED_INTEGRAL_LIMIT_MM_S 1000.0f
/* 世界系速度输出限幅，单位：mm/s */
#define CHASSIS_WORLD_SPEED_OUTPUT_LIMIT_MM_S   800.0f
/* 参考系重置时的指令阈值，单位：mm/s */
#define CHASSIS_WORLD_SPEED_RESET_CMD_MM_S      50.0f
/* 参考系重置时的实测阈值，单位：mm/s */
#define CHASSIS_WORLD_SPEED_RESET_MEAS_MM_S     80.0f

/* M3508 速度环参数 */
/* 电机速度环比例系数 */
#define M3508_SPEED_KP                 12.0f
/* 电机速度环积分系数 */
#define M3508_SPEED_KI                 0.15f
/* 电机速度环微分系数 */
#define M3508_SPEED_KD                 0.0f
/* 电机最大输出电流 */
#define M3508_CURRENT_LIMIT            12000.0f
/* 电机速度环积分限幅 */
#define M3508_INTEGRAL_LIMIT           4000.0f

/* DT7 三段开关编码 */
/* 三段开关上拨编码 */
#define DBUS_SWITCH_UP                 1U
/* 三段开关中拨编码 */
#define DBUS_SWITCH_MID                3U
/* 三段开关下拨编码 */
#define DBUS_SWITCH_DOWN               2U

#endif
