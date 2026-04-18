#include "chassis.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_config.h"
#include "task.h"

typedef struct
{
    float kp;
    float ki;
    float integral_output;
    float integral_limit;
    float output_limit;
} chassis_pi_t;

static const float k_deg_to_rad = 0.01745329252f;
static const float k_ctrl_period_s = (float)CTRL_TASK_PERIOD_MS * 0.001f;

static chassis_state_t chassis_state;
static chassis_pi_t s_world_vx_pi;
static chassis_pi_t s_world_vy_pi;
static bool s_reference_initialized = false;
static bool s_imu_ready_last = false;
static bool s_remote_lost_last = true;

static float chassis_limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static float chassis_wrap_angle(float angle_rad)
{
    const float pi = 3.1415926f;
    const float two_pi = 6.2831852f;

    while (angle_rad > pi)
    {
        angle_rad -= two_pi;
    }

    while (angle_rad < -pi)
    {
        angle_rad += two_pi;
    }

    return angle_rad;
}

static void chassis_rotate_vector(float x_in,
                                  float y_in,
                                  float angle_rad,
                                  float *x_out,
                                  float *y_out)
{
    const float cos_angle = cosf(angle_rad);
    const float sin_angle = sinf(angle_rad);

    *x_out = x_in * cos_angle + y_in * sin_angle;
    *y_out = -x_in * sin_angle + y_in * cos_angle;
}

static void chassis_limit_vector(float *x, float *y, float limit)
{
    const float magnitude = sqrtf((*x * *x) + (*y * *y));

    if ((limit <= 0.0f) || (magnitude <= limit) || (magnitude <= 1e-6f))
    {
        return;
    }

    *x *= limit / magnitude;
    *y *= limit / magnitude;
}

static void chassis_pi_init(chassis_pi_t *controller)
{
    controller->kp = CHASSIS_WORLD_SPEED_KP;
    controller->ki = CHASSIS_WORLD_SPEED_KI;
    controller->integral_output = 0.0f;
    controller->integral_limit = CHASSIS_WORLD_SPEED_INTEGRAL_LIMIT_MM_S;
    controller->output_limit = CHASSIS_WORLD_SPEED_OUTPUT_LIMIT_MM_S;
}

static void chassis_pi_reset(chassis_pi_t *controller)
{
    controller->integral_output = 0.0f;
}

static float chassis_pi_update(chassis_pi_t *controller, float error)
{
    controller->integral_output += controller->ki * error * k_ctrl_period_s;
    controller->integral_output = chassis_limit(controller->integral_output, controller->integral_limit);

    return chassis_limit((controller->kp * error) + controller->integral_output,
                         controller->output_limit);
}

static int16_t chassis_apply_deadband(int16_t value)
{
    if ((value < DBUS_CHANNEL_DEADBAND) && (value > -DBUS_CHANNEL_DEADBAND))
    {
        return 0;
    }

    return value;
}

static float chassis_scale_channel(int16_t channel, float max_value)
{
    float normalized;

    normalized = (float)chassis_apply_deadband(channel) / (float)(DBUS_CHANNEL_MAX - DBUS_CHANNEL_CENTER);
    normalized = chassis_limit(normalized, 1.0f);
    return normalized * max_value;
}

static chassis_mode_t chassis_get_mode_from_remote(const dbus_data_t *remote)
{
    switch (remote->s1)
    {
    case DBUS_SWITCH_UP:
        return CHASSIS_MODE_NORMAL;

    case DBUS_SWITCH_DOWN:
        return CHASSIS_MODE_GYROSCOPE;

    case DBUS_SWITCH_MID:
    default:
        return CHASSIS_MODE_STOP;
    }
}

static float chassis_get_speed_limit_from_remote(const dbus_data_t *remote,
                                                 chassis_mode_t *mode,
                                                 bool *output_enabled)
{
    switch (remote->s2)
    {
    case DBUS_SWITCH_UP:
        *output_enabled = false;
        return 0.0f;

    case DBUS_SWITCH_MID:
        *output_enabled = true;
        return CHASSIS_MAX_LINEAR_SPEED_MM_S * CHASSIS_S2_SPEED_RATIO_MID;

    case DBUS_SWITCH_DOWN:
        *output_enabled = true;
        return CHASSIS_MAX_LINEAR_SPEED_MM_S * CHASSIS_S2_SPEED_RATIO_DOWN;

    default:
        *output_enabled = false;
        *mode = CHASSIS_MODE_STOP;
        return 0.0f;
    }
}

static void chassis_align_reference(chassis_state_t *state, float body_yaw_world_rad)
{
    state->body_yaw_world_rad = body_yaw_world_rad;
    state->head_yaw_world_rad = body_yaw_world_rad;
    state->body_yaw_target_rad = body_yaw_world_rad;
}

static void chassis_reset_translation_controllers(void)
{
    chassis_pi_reset(&s_world_vx_pi);
    chassis_pi_reset(&s_world_vy_pi);
}

static void chassis_clear_motion_outputs(chassis_state_t *state)
{
    state->vx_mm_s = 0.0f;
    state->vy_mm_s = 0.0f;
    state->wz_rad_s = 0.0f;
    state->vx_ref_world_mm_s = 0.0f;
    state->vy_ref_world_mm_s = 0.0f;
    state->wheel_rpm[0] = 0;
    state->wheel_rpm[1] = 0;
    state->wheel_rpm[2] = 0;
    state->wheel_rpm[3] = 0;
}

static void chassis_enter_output_disabled_state(chassis_state_t *state,
                                                float body_yaw_world_rad,
                                                bool align_reference)
{
    chassis_reset_translation_controllers();
    chassis_clear_motion_outputs(state);
    if (align_reference)
    {
        chassis_align_reference(state, body_yaw_world_rad);
    }
    else
    {
        state->body_yaw_target_rad = body_yaw_world_rad;
    }
    state->vx_cmd_head_mm_s = 0.0f;
    state->vy_cmd_head_mm_s = 0.0f;
    state->vx_cmd_world_mm_s = 0.0f;
    state->vy_cmd_world_mm_s = 0.0f;
    state->vx_meas_body_mm_s = 0.0f;
    state->vy_meas_body_mm_s = 0.0f;
    state->vx_meas_world_mm_s = 0.0f;
    state->vy_meas_world_mm_s = 0.0f;
    state->wz_meas_rad_s = 0.0f;
    state->mode = CHASSIS_MODE_STOP;
    state->output_enabled = 0U;
}

static void chassis_estimate_body_velocity(chassis_state_t *state, const int16_t wheel_feedback_rpm[4])
{
    float wheel_linear_mm_s[4];
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        wheel_linear_mm_s[i] = ((float)wheel_feedback_rpm[i] * CHASSIS_WHEEL_CIRCUM_MM) /
                               (60.0f * CHASSIS_GEAR_RATIO);
    }

    state->vx_meas_body_mm_s = (wheel_linear_mm_s[0] + wheel_linear_mm_s[1] -
                                wheel_linear_mm_s[2] - wheel_linear_mm_s[3]) * 0.25f;
    state->vy_meas_body_mm_s = (wheel_linear_mm_s[0] - wheel_linear_mm_s[1] -
                                wheel_linear_mm_s[2] + wheel_linear_mm_s[3]) * 0.25f;
}

static void chassis_calculate_wheel_rpm(chassis_state_t *state)
{
    const float ratio = 60.0f / CHASSIS_WHEEL_CIRCUM_MM * CHASSIS_GEAR_RATIO;
    const float rotate_term = state->wz_rad_s * CHASSIS_ROTATE_RADIUS_MM;

    state->wheel_rpm[0] = (int16_t)((state->vx_mm_s + state->vy_mm_s + rotate_term) * ratio);
    state->wheel_rpm[1] = (int16_t)((state->vx_mm_s - state->vy_mm_s + rotate_term) * ratio);
    state->wheel_rpm[2] = (int16_t)((-state->vx_mm_s - state->vy_mm_s + rotate_term) * ratio);
    state->wheel_rpm[3] = (int16_t)((-state->vx_mm_s + state->vy_mm_s + rotate_term) * ratio);
}

void chassis_init(void)
{
    memset(&chassis_state, 0, sizeof(chassis_state));
    chassis_state.mode = CHASSIS_MODE_STOP;
    chassis_state.chassis_max_limit_speed_mm_s = CHASSIS_MAX_LINEAR_SPEED_MM_S;
    chassis_state.output_enabled = 0U;
    chassis_pi_init(&s_world_vx_pi);
    chassis_pi_init(&s_world_vy_pi);
    s_reference_initialized = false;
    s_imu_ready_last = false;
    s_remote_lost_last = true;
}

void chassis_reset(void)
{
    chassis_state_t next_state = chassis_state;

    chassis_enter_output_disabled_state(&next_state, next_state.body_yaw_world_rad, true);

    taskENTER_CRITICAL();
    chassis_state = next_state;
    taskEXIT_CRITICAL();
}

void chassis_control_step(const dbus_data_t *remote,
                          const imu_data_t *imu,
                          const int16_t wheel_feedback_rpm[4])
{
    chassis_state_t next_state = chassis_state;
    chassis_mode_t mode;
    bool imu_ready;
    bool remote_lost;
    bool output_enabled;
    float speed_limit;
    float vx_ref_world_mm_s;
    float vy_ref_world_mm_s;

    if ((remote == NULL) || (imu == NULL) || (wheel_feedback_rpm == NULL))
    {
        return;
    }

    imu_ready = imu_is_ready();
    remote_lost = (bsp_dbus_is_lost(DBUS_LOSS_TIMEOUT_MS) != 0U);
    mode = chassis_get_mode_from_remote(remote);
    speed_limit = chassis_get_speed_limit_from_remote(remote, &mode, &output_enabled);

    if (!imu_ready)
    {
        s_reference_initialized = false;
        s_imu_ready_last = false;
        s_remote_lost_last = remote_lost;
        next_state.chassis_max_limit_speed_mm_s = 0.0f;
        chassis_enter_output_disabled_state(&next_state, next_state.body_yaw_world_rad, false);

        taskENTER_CRITICAL();
        chassis_state = next_state;
        taskEXIT_CRITICAL();
        return;
    }

    next_state.body_yaw_world_rad = chassis_wrap_angle(imu->yaw * k_deg_to_rad);
    next_state.wz_meas_rad_s = imu->wz;

    if ((!s_imu_ready_last) || (!s_reference_initialized))
    {
        chassis_align_reference(&next_state, next_state.body_yaw_world_rad);
        s_reference_initialized = true;
    }

    s_imu_ready_last = true;
    next_state.chassis_max_limit_speed_mm_s = speed_limit;
    next_state.output_enabled = output_enabled ? 1U : 0U;

    next_state.head_yaw_world_rad = chassis_wrap_angle(next_state.head_yaw_world_rad +
                                                       chassis_scale_channel(remote->ch2,
                                                                             CHASSIS_MAX_HEAD_YAW_RATE_RAD_S) *
                                                       k_ctrl_period_s);
    next_state.vx_cmd_head_mm_s = chassis_scale_channel(remote->ch1, speed_limit);
    next_state.vy_cmd_head_mm_s = chassis_scale_channel(remote->ch0, speed_limit);
    chassis_rotate_vector(next_state.vx_cmd_head_mm_s,
                          next_state.vy_cmd_head_mm_s,
                          next_state.head_yaw_world_rad,
                          &next_state.vx_cmd_world_mm_s,
                          &next_state.vy_cmd_world_mm_s);

    chassis_estimate_body_velocity(&next_state, wheel_feedback_rpm);
    chassis_rotate_vector(next_state.vx_meas_body_mm_s,
                          next_state.vy_meas_body_mm_s,
                          next_state.body_yaw_world_rad,
                          &next_state.vx_meas_world_mm_s,
                          &next_state.vy_meas_world_mm_s);

    if (remote_lost)
    {
        const bool align_reference = !s_remote_lost_last;

        s_remote_lost_last = true;
        next_state.chassis_max_limit_speed_mm_s = 0.0f;
        chassis_enter_output_disabled_state(&next_state, next_state.body_yaw_world_rad, align_reference);

        taskENTER_CRITICAL();
        chassis_state = next_state;
        taskEXIT_CRITICAL();
        return;
    }

    s_remote_lost_last = false;
    next_state.mode = mode;

    if (!output_enabled)
    {
        next_state.chassis_max_limit_speed_mm_s = 0.0f;
        chassis_enter_output_disabled_state(&next_state, next_state.body_yaw_world_rad, true);

        taskENTER_CRITICAL();
        chassis_state = next_state;
        taskEXIT_CRITICAL();
        return;
    }

    if (mode == CHASSIS_MODE_STOP)
    {
        chassis_reset_translation_controllers();
        next_state.body_yaw_target_rad = next_state.body_yaw_world_rad;
        chassis_clear_motion_outputs(&next_state);
        next_state.output_enabled = 1U;

        taskENTER_CRITICAL();
        chassis_state = next_state;
        taskEXIT_CRITICAL();
        return;
    }

    if ((fabsf(next_state.vx_cmd_world_mm_s) < CHASSIS_WORLD_SPEED_RESET_CMD_MM_S) &&
        (fabsf(next_state.vy_cmd_world_mm_s) < CHASSIS_WORLD_SPEED_RESET_CMD_MM_S) &&
        (fabsf(next_state.vx_meas_world_mm_s) < CHASSIS_WORLD_SPEED_RESET_MEAS_MM_S) &&
        (fabsf(next_state.vy_meas_world_mm_s) < CHASSIS_WORLD_SPEED_RESET_MEAS_MM_S))
    {
        chassis_reset_translation_controllers();
    }

    vx_ref_world_mm_s = next_state.vx_cmd_world_mm_s +
                        chassis_pi_update(&s_world_vx_pi,
                                          next_state.vx_cmd_world_mm_s - next_state.vx_meas_world_mm_s);
    vy_ref_world_mm_s = next_state.vy_cmd_world_mm_s +
                        chassis_pi_update(&s_world_vy_pi,
                                          next_state.vy_cmd_world_mm_s - next_state.vy_meas_world_mm_s);
    chassis_limit_vector(&vx_ref_world_mm_s, &vy_ref_world_mm_s, speed_limit);

    next_state.vx_ref_world_mm_s = vx_ref_world_mm_s;
    next_state.vy_ref_world_mm_s = vy_ref_world_mm_s;
    chassis_rotate_vector(next_state.vx_ref_world_mm_s,
                          next_state.vy_ref_world_mm_s,
                          -next_state.body_yaw_world_rad,
                          &next_state.vx_mm_s,
                          &next_state.vy_mm_s);

    if (mode == CHASSIS_MODE_NORMAL)
    {
        next_state.body_yaw_target_rad = next_state.head_yaw_world_rad;
        next_state.wz_rad_s = chassis_limit(CHASSIS_YAW_HOLD_KP *
                                            chassis_wrap_angle(next_state.body_yaw_target_rad -
                                                               next_state.body_yaw_world_rad) -
                                            CHASSIS_YAW_HOLD_KD * next_state.wz_meas_rad_s,
                                            CHASSIS_MAX_ANGULAR_SPEED_RAD);
    }
    else
    {
        next_state.body_yaw_target_rad = next_state.body_yaw_world_rad;
        next_state.wz_rad_s = CHASSIS_GYROSCOPE_SPIN_SPEED_RAD_S;
    }

    chassis_calculate_wheel_rpm(&next_state);

    taskENTER_CRITICAL();
    chassis_state = next_state;
    taskEXIT_CRITICAL();
}

void chassis_get_wheel_rpm(int16_t wheel_rpm[4])
{
    if (wheel_rpm == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    memcpy(wheel_rpm, chassis_state.wheel_rpm, sizeof(chassis_state.wheel_rpm));
    taskEXIT_CRITICAL();
}

uint8_t chassis_is_output_enabled(void)
{
    uint8_t output_enabled;

    taskENTER_CRITICAL();
    output_enabled = chassis_state.output_enabled;
    taskEXIT_CRITICAL();

    return output_enabled;
}

void chassis_get_state(chassis_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *state = chassis_state;
    taskEXIT_CRITICAL();
}
