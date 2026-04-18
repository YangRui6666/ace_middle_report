#include "bsp_dbus.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_config.h"
#include "main.h"
#include "task.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t dbus_rx_buffer[DBUS_FRAME_SIZE];
static dbus_data_t dbus_data;

/**
 * @brief 将原始通道值转换为中值归零后的控制量
 * @param[in] raw_value DBUS 原始通道值
 * @retval 返回减去中值 1024 的结果
 */
static int16_t dbus_decode_channel(uint16_t raw_value)
{
    return (int16_t)raw_value - DBUS_CHANNEL_CENTER;
}

/**
 * @brief 解码一帧完整的 DT7 数据
 * @param[in] none
 * @retval none
 * @attention
 * 该函数只负责解包和更新快照，不做模式判断和控制映射。
 */
static void dbus_decode_frame(void)
{
    dbus_data_t temp;
    UBaseType_t interrupt_state;

    memset(&temp, 0, sizeof(temp));

    temp.ch0 = dbus_decode_channel((uint16_t)(dbus_rx_buffer[0] | ((dbus_rx_buffer[1] & 0x07U) << 8)));
    temp.ch1 = dbus_decode_channel((uint16_t)(((dbus_rx_buffer[1] & 0xF8U) >> 3) | ((dbus_rx_buffer[2] & 0x3FU) << 5)));
    temp.ch2 = dbus_decode_channel((uint16_t)(((dbus_rx_buffer[2] & 0xC0U) >> 6) | (dbus_rx_buffer[3] << 2) |
                                              ((dbus_rx_buffer[4] & 0x01U) << 10)));
    temp.ch3 = dbus_decode_channel((uint16_t)(((dbus_rx_buffer[4] & 0xFEU) >> 1) | ((dbus_rx_buffer[5] & 0x0FU) << 7)));
    temp.s1 = (dbus_rx_buffer[5] >> 6) & 0x03U;
    temp.s2 = (dbus_rx_buffer[5] >> 4) & 0x03U;
    temp.mouse_x = (int16_t)(dbus_rx_buffer[6] | (dbus_rx_buffer[7] << 8));
    temp.mouse_y = (int16_t)(dbus_rx_buffer[8] | (dbus_rx_buffer[9] << 8));
    temp.mouse_z = (int16_t)(dbus_rx_buffer[10] | (dbus_rx_buffer[11] << 8));
    temp.mouse_left = dbus_rx_buffer[12];
    temp.mouse_right = dbus_rx_buffer[13];
    temp.keyboard = (uint16_t)(dbus_rx_buffer[14] | (dbus_rx_buffer[15] << 8));
    temp.wheel = dbus_decode_channel((uint16_t)(dbus_rx_buffer[16] | (dbus_rx_buffer[17] << 8)));
    temp.online = 1U;
    temp.last_update_tick = xTaskGetTickCountFromISR();

    interrupt_state = taskENTER_CRITICAL_FROM_ISR();
    dbus_data = temp;
    taskEXIT_CRITICAL_FROM_ISR(interrupt_state);
}

/**
 * @brief 初始化 DT7 接收
 * @param[in] none
 * @retval none
 * @attention
 * 使用 USART3 DMA 持续接收，并依赖空闲中断判定一帧结束。
 */
void bsp_dbus_init(void)
{
    memset(dbus_rx_buffer, 0, sizeof(dbus_rx_buffer));
    memset(&dbus_data, 0, sizeof(dbus_data));

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, dbus_rx_buffer, DBUS_FRAME_SIZE);
}

/**
 * @brief USART3 空闲中断中的 DT7 收包处理
 * @param[in] none
 * @retval none
 * @attention
 * 只有 DMA 计数归零时才认为当前帧完整，避免半帧数据污染控制输入。
 */
void bsp_dbus_irq_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == RESET)
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    HAL_UART_DMAStop(&huart3);

    if (__HAL_DMA_GET_COUNTER(&hdma_usart3_rx) == 0U)
    {
        dbus_decode_frame();
    }

    HAL_UART_Receive_DMA(&huart3, dbus_rx_buffer, DBUS_FRAME_SIZE);
}

/**
 * @brief 获取遥控器数据快照
 * @param[out] data 遥控器数据输出缓存
 * @retval none
 */
void bsp_dbus_get_data(dbus_data_t *data)
{
    taskENTER_CRITICAL();
    *data = dbus_data;
    taskEXIT_CRITICAL();
}

/**
 * @brief 判断遥控器是否失联
 * @param[in] timeout_ms 超时时间，单位 ms
 * @retval 1 表示已失联，0 表示仍在线
 */
uint8_t bsp_dbus_is_lost(uint32_t timeout_ms)
{
    dbus_data_t snapshot;
    TickType_t now;
    TickType_t elapsed;

    taskENTER_CRITICAL();
    snapshot = dbus_data;
    taskEXIT_CRITICAL();

    if (snapshot.online == 0U)
    {
        return 1U;
    }

    now = xTaskGetTickCount();
    elapsed = now - snapshot.last_update_tick;
    return (elapsed > (TickType_t)timeout_ms) ? 1U : 0U;
}
