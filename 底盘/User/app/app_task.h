#ifndef APP_TASK_H
#define APP_TASK_H

/**
 * @brief 控制任务入口
 * @param[in] argument 任务参数，当前未使用
 * @retval none
 * @attention
 * 该任务负责读取遥控器数据并刷新底盘目标，不直接操作 CAN 电流输出。
 */
void ctrlStartTask(void *argument);

/**
 * @brief 电机任务入口
 * @param[in] argument 任务参数，当前未使用
 * @retval none
 * @attention
 * 该任务负责执行底盘电机速度环并通过 CAN1 发送电流。
 */
void motorStartTask(void *argument);

#endif
