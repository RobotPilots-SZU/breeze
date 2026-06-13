/*
 * Copyright (c) 2026 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0 
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT rp_servo_motor_pwm  

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <drivers/servo_motor_pwm.h>

#define LOG_LEVEL CONFIG_SERVO_MOTOR_PWM_LOG_LEVEL
LOG_MODULE_REGISTER(servo_motor_pwm);


typedef struct servo_motor_pwm_cfg {
    const struct pwm_dt_spec pwm;
    uint32_t min_pulse;
    uint32_t max_pulse;
    uint16_t min_angle;
    uint16_t max_angle;
} servo_motor_pwm_cfg_t;

typedef struct servo_motor_pwm_data {
    float logical_angle;
    float physical_angle;
    float up_angle;
    float down_angle;
    float offset_angle;
} servo_motor_pwm_data_t;

static uint32_t angle_to_pulse(const struct device *servo, float angle);


/**
 * @brief Set the mechanical limit angle of the servo motor.
 * @param servo servo motor device
 * @param up_angle upper mechanical limit angle in degrees
 * @param down_angle lower mechanical limit angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static int servo_mechanical_limit_angle_fn(const struct device *servo, float up_angle, float down_angle)
{
    const servo_motor_pwm_cfg_t *cfg = servo->config;
    servo_motor_pwm_data_t *data = servo->data;

    if (up_angle > (float)cfg->max_angle || down_angle < (float)cfg->min_angle || up_angle <= down_angle)
    {
        LOG_ERR("Invalid mechanical limits: up_angle %f, down_angle %f, max_angle %u, min_angle %u",
                (double)up_angle, (double)down_angle, cfg->max_angle, cfg->min_angle);
        return -EINVAL;
    }
    else
    {
        data->up_angle = up_angle;
        data->down_angle = down_angle;
    }
    LOG_INF("Mechanical limits set: up_angle %f, down_angle %f", (double)up_angle, (double)down_angle);
    return 0;
}

/**
 * @brief Set the offset angle of the servo motor. 
 *        (physical_angle = logical_angle + offset_angle)
 * @param servo servo motor device
 * @param offset_angle offset angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static int servo_set_offset_angle_fn(const struct device *servo, float offset_angle)
{
    const servo_motor_pwm_cfg_t *cfg = servo->config;
    servo_motor_pwm_data_t *data = servo->data;

    if ((offset_angle - cfg->min_angle) > (data->up_angle - cfg->min_angle) || (offset_angle - cfg->min_angle) < (data->down_angle - cfg->min_angle))
    {
        LOG_ERR("Invalid offset angle %f, up_angle %f, down_angle %f",
                (double)offset_angle, (double)data->up_angle, (double)data->down_angle);
        return -EINVAL;
    }
    data->offset_angle = offset_angle;
    data->logical_angle = (float)0;
    data->physical_angle = (float)data->logical_angle + offset_angle;
    
    LOG_INF("Offset angle set: %f", (double)offset_angle);
    return 0;
}

/**
 * @brief Set the angle of the servo motor.
 * @param servo servo motor device
 * @param angle motor angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static int servo_set_angle_fn(const struct device *servo, float angle) 
{
    const servo_motor_pwm_cfg_t *cfg = servo->config;
    servo_motor_pwm_data_t *data = servo->data;
    float output_angle = angle + data->offset_angle;

    if (output_angle > data->up_angle)
    {
        LOG_WRN("Angle %f exceeds up_angle %f, clamping", (double)output_angle, (double)data->up_angle);
        output_angle = data->up_angle;
    }
    else if (output_angle < data->down_angle)
    {
        LOG_WRN("Angle %f below down_angle %f, clamping", (double)output_angle, (double)data->down_angle);
        output_angle = data->down_angle;
    }

    uint32_t pulse = angle_to_pulse(servo, output_angle);
    int ret = pwm_set_pulse_dt(&cfg->pwm, pulse);
    if (ret == 0) {
        data->logical_angle = output_angle - (float)data->offset_angle;
        data->physical_angle = output_angle;
        LOG_DBG("Set angle to %f (pulse %u ns)", (double)(output_angle - data->offset_angle), pulse);
    } 
    else 
    {
        LOG_ERR("Failed to set PWM pulse (err %d)", ret);
    }
    return ret;
}

/**
 * @brief Rotate the servo by a relative angle offset.
 * @param servo servo motor device
 * @param direction rotation direction:
 *                  - positive (1): towards max_angle
 *                  - negative (-1): towards min_angle
 * @param rotate_angle rotation angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static int servo_rotate_angle_fn(const struct device *servo, int8_t direction, float rotate_angle)
{
    const servo_motor_pwm_cfg_t *cfg = servo->config;
    servo_motor_pwm_data_t *data = servo->data;

    int8_t dir = direction;
    float target_angle = dir * rotate_angle + data->logical_angle;
    float output_angle = target_angle + data->offset_angle;

    if ((dir != 1) && (dir != -1))
    {
        LOG_ERR("Invalid direction %d, only 1 or -1 allowed", dir);
        return -EINVAL;
    }

    if (output_angle > data->up_angle)
    {
        LOG_WRN("Target angle %f exceeds up_angle %f, clamping", (double)output_angle, (double)data->up_angle);
        target_angle = data->up_angle - data->offset_angle;
        output_angle = data->up_angle;
    }
    else if (output_angle < data->down_angle)
    {
        LOG_WRN("Target angle %f below down_angle %f, clamping", (double)output_angle, (double)data->down_angle);
        target_angle = data->down_angle - data->offset_angle;
        output_angle = data->down_angle;
    }

    uint32_t pulse = angle_to_pulse(servo, output_angle);
    int ret = pwm_set_pulse_dt(&cfg->pwm, pulse);
    if (ret == 0) {
        data->logical_angle = target_angle;
        data->physical_angle = output_angle;
        LOG_DBG("Rotating angle by %f degrees to target %f (physical angle %f)",
            (double)(dir * rotate_angle), (double)target_angle, (double)data->physical_angle);
    } 
    else 
    {
        LOG_ERR("Failed to rotate angle (err %d)", ret);
    }
    return ret;
}

static const struct servo_motor_pwm_api servo_pwm_api = {
    .mechanical_limit_angle = servo_mechanical_limit_angle_fn,
    .offset_angle = servo_set_offset_angle_fn,
    .set_angle = servo_set_angle_fn,
    .rotate_angle = servo_rotate_angle_fn,
};


/**
 * @brief Convert angle to pulse width in nanoseconds.
 * @param servo servo motor device
 * @param angle motor angle in degrees
 * @retval uint32_t pulse width in nanoseconds
 */
static uint32_t angle_to_pulse(const struct device *servo, float angle)
{
    const servo_motor_pwm_cfg_t *cfg = servo->config;
    servo_motor_pwm_data_t *data = servo->data;

    angle = CLAMP(angle, data->down_angle, data->up_angle);

    uint32_t range_pulse = cfg->max_pulse - cfg->min_pulse;
    float range_angle = (float)(cfg->max_angle - cfg->min_angle);
    return (uint32_t)(cfg->min_pulse + (range_pulse * (angle - (float)cfg->min_angle) / range_angle));
}

/**
 * @brief Initialize the servo motor device.
 * @param servo servo motor device
 * @retval int 0 on success.
 */
static int servo_motor_pwm_init(const struct device *servo)
{
    const servo_motor_pwm_cfg_t *cfg = servo->config;
    servo_motor_pwm_data_t *data = servo->data;
    const struct pwm_dt_spec *pwm_spec = &cfg->pwm;

    __ASSERT(device_is_ready(pwm_spec->dev),
        "PWM controller for servo not ready. Check device tree and clock setup.");

    uint64_t cycles_per_sec = 0;
    int err = pwm_get_cycles_per_sec(pwm_spec->dev, pwm_spec->channel, &cycles_per_sec);
    __ASSERT((err >= 0) && (cycles_per_sec != 0),
        "pwm_get_cycles_per_sec failed or returned 0. Check PWM clock config.");

    __ASSERT(cfg->min_pulse < cfg->max_pulse,
        "min-pulse (%u ns) must be less than max-pulse (%u ns)",
        cfg->min_pulse, cfg->max_pulse);
    __ASSERT(cfg->max_pulse < pwm_spec->period,
        "max-pulse (%u ns) must be less than PWM period (%u ns)",
        cfg->max_pulse, pwm_spec->period);
    __ASSERT(cfg->min_pulse >= 500000 && cfg->max_pulse <= 2500000,
        "min-pulse (%u ns) must be greater than or equal to 500000 ns" 
        " and max-pulse (%u ns) must be less than or equal to 2500000 ns",
        cfg->min_pulse, cfg->max_pulse);

    __ASSERT(cfg->max_angle > cfg->min_angle && cfg->max_angle <= 360 && cfg->min_angle >= 0,
             "Invalid angle range: max-angle must be less than or equal to 360 (got %u), "
             "min-angle must be greater than or equal to 0 (got %u), "
             "and max-angle must be greater than min-angle",
             cfg->max_angle, cfg->min_angle);

    LOG_INF("Servo initialized: dev=%p, channel=%u, period=%u ns, "
            "min_pulse=%u ns, max_pulse=%u ns, max_angle=%u, min_angle=%u",
            servo, pwm_spec->channel, pwm_spec->period,
            cfg->min_pulse, cfg->max_pulse, cfg->max_angle, cfg->min_angle);

    data->offset_angle = 0;
    data->down_angle = cfg->min_angle;
    data->up_angle = cfg->max_angle;
    data->logical_angle = cfg->min_angle;
    data->physical_angle = cfg->min_angle;

    return 0;
}


#define SERVO_MOTOR_PWM_INIT(inst) \
    BUILD_ASSERT(DT_INST_PROP(inst, min_pulse) < DT_INST_PROP(inst, max_pulse), \
                 "min-pulse must be less than max-pulse");                      \
    \
    static servo_motor_pwm_data_t servo_data_##inst = { \
        .logical_angle = 0,                             \
        .physical_angle = 0,                          \
        .up_angle = 0,                                  \
        .down_angle = 0,                                \
        .offset_angle = 0,                              \
    }; \
    \
    static const servo_motor_pwm_cfg_t servo_config_##inst = {  \
        .pwm = PWM_DT_SPEC_INST_GET(inst),                      \
        .min_pulse = DT_INST_PROP(inst, min_pulse),             \
        .max_pulse = DT_INST_PROP(inst, max_pulse),             \
        .max_angle = DT_INST_PROP(inst, max_angle),             \
        .min_angle = DT_INST_PROP_OR(inst, min_angle, 0),       \
    }; \
    DEVICE_DT_INST_DEFINE(inst,                                     \
                          servo_motor_pwm_init,                     \
                          NULL,                                     \
                          &servo_data_##inst,                       \
                          &servo_config_##inst,                     \
                          POST_KERNEL,                              \
                          CONFIG_SERVO_INIT_PRIORITY,               \
                          &servo_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(SERVO_MOTOR_PWM_INIT);
