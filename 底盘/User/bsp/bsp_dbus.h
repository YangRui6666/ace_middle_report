#ifndef BSP_DBUS_H
#define BSP_DBUS_H

#include <stdint.h>

/**
 * @brief DT7 遥控器解码后的数据
 * @attention
 * 通道值已减去 1024 中值，可直接作为控制输入使用。
 * s1、s2 保留原始三段开关编码，online 表示最近一次收包成功。
 */
typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
    int16_t wheel;
    uint8_t online;
    uint32_t last_update_tick;
} dbus_data_t;

/**
 * @brief 初始化 DT7 接收模块
 * @param[in] none
 * @retval none
 * @attention
 * 当前固定使用 USART3 + DMA + 空闲中断收包。
 */
void bsp_dbus_init(void);

/**
 * @brief USART3 空闲中断处理入口
 * @param[in] none
 * @retval none
 * @attention
 * 仅在检测到完整 18 字节帧时更新解码结果。
 */
void bsp_dbus_irq_handler(void);

/**
 * @brief 获取最近一次解码成功的遥控器数据
 * @param[out] data 遥控器数据输出缓存
 * @retval none
 */
void bsp_dbus_get_data(dbus_data_t *data);

/**
 * @brief 判断遥控器最近一次有效帧是否超时
 * @param[in] timeout_ms 超时时间，单位 ms
 * @retval 1 表示已超时，0 表示仍在线
 */
uint8_t bsp_dbus_is_lost(uint32_t timeout_ms);

#endif
