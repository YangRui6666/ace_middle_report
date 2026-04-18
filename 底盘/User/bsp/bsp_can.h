#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>

/**
 * @brief 单个 M3508 电机的 CAN 反馈数据
 */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    uint8_t online;
} can_motor_feedback_t;

/**
 * @brief 初始化 CAN1 过滤器和接收中断
 * @param[in] none
 * @retval none
 */
void bsp_can_init(void);

/**
 * @brief 发送四个底盘电机的目标电流
 * @param[in] iq1 1 号电机目标电流
 * @param[in] iq2 2 号电机目标电流
 * @param[in] iq3 3 号电机目标电流
 * @param[in] iq4 4 号电机目标电流
 * @retval none
 * @attention
 * 当前使用标准 ID 0x200，对应 0x201~0x204 四个 M3508。
 */
void bsp_can_set_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

/**
 * @brief 获取指定电机的最近一次反馈
 * @param[in] index 电机序号，范围为 1~4
 * @param[out] feedback 电机反馈输出缓存
 * @retval none
 */
void bsp_can_get_motor_feedback(uint8_t index, can_motor_feedback_t *feedback);

#endif
