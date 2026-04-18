#include "bsp_can.h"

#include <string.h>

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

extern CAN_HandleTypeDef hcan1;

static can_motor_feedback_t motor_feedback[4];

/**
 * @brief 初始化 CAN1 过滤器和接收通知
 * @param[in] none
 * @retval none
 * @attention
 * 当前不过滤 ID，统一收进 FIFO0，再在回调中筛选 0x201~0x204。
 */
void bsp_can_init(void)
{
    CAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief 发送底盘四个电机的目标电流
 * @param[in] iq1 1 号电机目标电流
 * @param[in] iq2 2 号电机目标电流
 * @param[in] iq3 3 号电机目标电流
 * @param[in] iq4 4 号电机目标电流
 * @retval none
 */
void bsp_can_set_chassis_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    uint8_t data[8];

    tx_header.StdId = 0x200U;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    data[0] = (uint8_t)(iq1 >> 8);
    data[1] = (uint8_t)iq1;
    data[2] = (uint8_t)(iq2 >> 8);
    data[3] = (uint8_t)iq2;
    data[4] = (uint8_t)(iq3 >> 8);
    data[5] = (uint8_t)iq3;
    data[6] = (uint8_t)(iq4 >> 8);
    data[7] = (uint8_t)iq4;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

/**
 * @brief 获取指定电机最近一次反馈
 * @param[in] index 电机序号，范围为 1~4
 * @param[out] feedback 电机反馈输出缓存
 * @retval none
 */
void bsp_can_get_motor_feedback(uint8_t index, can_motor_feedback_t *feedback)
{
    if ((index < 1U) || (index > 4U))
    {
        memset(feedback, 0, sizeof(*feedback));
        return;
    }

    taskENTER_CRITICAL();
    *feedback = motor_feedback[index - 1U];
    taskEXIT_CRITICAL();
}

/**
 * @brief CAN FIFO0 接收回调
 * @param[in] hcan CAN 句柄
 * @retval none
 * @attention
 * 当前只处理底盘 4 个 M3508 的标准 ID 反馈，其他报文直接忽略。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    uint8_t index;
    UBaseType_t interrupt_state;

    if (hcan->Instance != CAN1)
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
        return;
    }

    if ((rx_header.StdId < 0x201U) || (rx_header.StdId > 0x204U))
    {
        return;
    }

    index = (uint8_t)(rx_header.StdId - 0x201U);

    interrupt_state = taskENTER_CRITICAL_FROM_ISR();
    motor_feedback[index].ecd = (uint16_t)((data[0] << 8) | data[1]);
    motor_feedback[index].speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor_feedback[index].given_current = (int16_t)((data[4] << 8) | data[5]);
    motor_feedback[index].temperature = data[6];
    motor_feedback[index].online = 1U;
    taskEXIT_CRITICAL_FROM_ISR(interrupt_state);
}
