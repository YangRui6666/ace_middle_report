#include "mahony_ahrs.h"

#include <math.h>
#include <stddef.h>

static float mahony_inv_sqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(value);
}

void mahony_ahrs_init(mahony_ahrs_t *ahrs)
{
    if (ahrs == NULL)
    {
        return;
    }

    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->two_kp = 2.0f;
    ahrs->two_ki = 0.0f;
    ahrs->integral_fb[0] = 0.0f;
    ahrs->integral_fb[1] = 0.0f;
    ahrs->integral_fb[2] = 0.0f;
}

void mahony_ahrs_update_imu(mahony_ahrs_t *ahrs,
                            float gx,
                            float gy,
                            float gz,
                            float ax,
                            float ay,
                            float az,
                            float dt)
{
    float recip_norm;
    float half_vx;
    float half_vy;
    float half_vz;
    float half_ex;
    float half_ey;
    float half_ez;
    float qa;
    float qb;
    float qc;
    float q0;
    float q1;
    float q2;
    float q3;

    if ((ahrs == NULL) || (dt <= 0.0f))
    {
        return;
    }

    q0 = ahrs->q0;
    q1 = ahrs->q1;
    q2 = ahrs->q2;
    q3 = ahrs->q3;

    if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
    {
        recip_norm = mahony_inv_sqrt(ax * ax + ay * ay + az * az);
        if (recip_norm > 0.0f)
        {
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            half_vx = q1 * q3 - q0 * q2;
            half_vy = q0 * q1 + q2 * q3;
            half_vz = q0 * q0 - 0.5f + q3 * q3;

            half_ex = (ay * half_vz) - (az * half_vy);
            half_ey = (az * half_vx) - (ax * half_vz);
            half_ez = (ax * half_vy) - (ay * half_vx);

            if (ahrs->two_ki > 0.0f)
            {
                ahrs->integral_fb[0] += ahrs->two_ki * half_ex * dt;
                ahrs->integral_fb[1] += ahrs->two_ki * half_ey * dt;
                ahrs->integral_fb[2] += ahrs->two_ki * half_ez * dt;
                gx += ahrs->integral_fb[0];
                gy += ahrs->integral_fb[1];
                gz += ahrs->integral_fb[2];
            }
            else
            {
                ahrs->integral_fb[0] = 0.0f;
                ahrs->integral_fb[1] = 0.0f;
                ahrs->integral_fb[2] = 0.0f;
            }

            gx += ahrs->two_kp * half_ex;
            gy += ahrs->two_kp * half_ey;
            gz += ahrs->two_kp * half_ez;
        }
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx) - (qc * gy) - (q3 * gz);
    q1 += (qa * gx) + (qc * gz) - (q3 * gy);
    q2 += (qa * gy) - (qb * gz) + (q3 * gx);
    q3 += (qa * gz) + (qb * gy) - (qc * gx);

    recip_norm = mahony_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (recip_norm <= 0.0f)
    {
        return;
    }

    ahrs->q0 = q0 * recip_norm;
    ahrs->q1 = q1 * recip_norm;
    ahrs->q2 = q2 * recip_norm;
    ahrs->q3 = q3 * recip_norm;
}

void mahony_ahrs_get_euler_deg(const mahony_ahrs_t *ahrs,
                               float *yaw,
                               float *pitch,
                               float *roll)
{
    const float rad_to_deg = 57.2957795131f;
    const float half_pi = 1.57079632679f;
    float q0;
    float q1;
    float q2;
    float q3;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float sin_yaw;
    float cos_yaw;

    if (ahrs == NULL)
    {
        return;
    }

    q0 = ahrs->q0;
    q1 = ahrs->q1;
    q2 = ahrs->q2;
    q3 = ahrs->q3;

    if (roll != NULL)
    {
        sin_roll = 2.0f * (q0 * q1 + q2 * q3);
        cos_roll = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        *roll = atan2f(sin_roll, cos_roll) * rad_to_deg;
    }

    if (pitch != NULL)
    {
        sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sin_pitch) >= 1.0f)
        {
            *pitch = copysignf(half_pi, sin_pitch) * rad_to_deg;
        }
        else
        {
            *pitch = asinf(sin_pitch) * rad_to_deg;
        }
    }

    if (yaw != NULL)
    {
        sin_yaw = 2.0f * (q0 * q3 + q1 * q2);
        cos_yaw = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        *yaw = atan2f(sin_yaw, cos_yaw) * rad_to_deg;
    }
}
