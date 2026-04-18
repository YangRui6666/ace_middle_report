#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float two_kp;
    float two_ki;
    float integral_fb[3];
} mahony_ahrs_t;

void mahony_ahrs_init(mahony_ahrs_t *ahrs);
void mahony_ahrs_update_imu(mahony_ahrs_t *ahrs,
                            float gx,
                            float gy,
                            float gz,
                            float ax,
                            float ay,
                            float az,
                            float dt);
void mahony_ahrs_get_euler_deg(const mahony_ahrs_t *ahrs,
                               float *yaw,
                               float *pitch,
                               float *roll);

#ifdef __cplusplus
}
#endif

#endif
