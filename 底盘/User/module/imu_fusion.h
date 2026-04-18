#ifndef GIMBAL_UM_IMU_FUSION_H
#define GIMBAL_UM_IMU_FUSION_H

#include <stdbool.h>

typedef struct imu_data_t
{
    float yaw;    // 单位：deg
    float pitch;  // 单位：deg
    float roll;   // 单位：deg
    float wz;     // 单位：rad/s
} imu_data_t;

#ifdef __cplusplus
extern "C" {
#endif

void imu_init(void);
void imu_update(void);
void imu_get_data(imu_data_t *imu_data);
bool imu_is_online(void);
bool imu_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif //GIMBAL_UM_IMU_FUSION_H
