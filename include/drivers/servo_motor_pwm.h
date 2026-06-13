/*
 * Copyright (c) 2026 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVO_MOTOR_PWM_H_
#define SERVO_MOTOR_PWM_H_

#include <zephyr/device.h>
#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Set the mechanical limit angle of the servo motor.
 * @param servo servo motor device
 * @param up_angle upper mechanical limit angle in degrees
 * @param down_angle lower mechanical limit angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
typedef int (*servo_motor_pwm_api_mechanical_limit_angle)(const struct device *servo, float up_angle, float down_angle);

/**
 * @brief Set the offset angle of the servo motor.
 * @param servo servo motor device
 * @param offset_angle offset angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
typedef int (*servo_motor_pwm_api_offset_angle)(const struct device *servo, float offset_angle);

/**
 * @brief Set the angle of the servo motor.
 * @param servo servo motor device
 * @param angle motor angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
typedef int (*servo_motor_pwm_api_set_angle)(const struct device *servo, float angle);

/**
 * @brief Rotate the servo by a relative angle offset.
 * @param servo servo motor device
 * @param direction rotation direction (1 or -1 for clockwise or counterclockwise)
 * @param rotate_angle rotation angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
typedef int (*servo_motor_pwm_api_rotate_angle)(const struct device *servo, int8_t direction, float rotate_angle);


struct servo_motor_pwm_api 
{
    servo_motor_pwm_api_mechanical_limit_angle mechanical_limit_angle;
    servo_motor_pwm_api_offset_angle offset_angle;
    servo_motor_pwm_api_set_angle set_angle;
    servo_motor_pwm_api_rotate_angle rotate_angle;
};


/**
 * @brief Set the mechanical limit angle of the servo motor.
 * @param servo servo motor device
 * @param up_angle upper mechanical limit angle in degrees
 * @param down_angle lower mechanical limit angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static inline int servo_set_mechanical_limit_angle(const struct device *servo, float up_angle, float down_angle) 
{
    const struct servo_motor_pwm_api *api = (const struct servo_motor_pwm_api *)servo->api;
    if (api->mechanical_limit_angle == NULL) 
    {
        return -ENOSYS;
    }
    return api->mechanical_limit_angle(servo, up_angle, down_angle);
}

/**
 * @brief Set the offset angle of the servo motor.
 * @param servo servo motor device
 * @param offset_angle offset angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static inline int servo_set_offset_angle(const struct device *servo, float offset_angle) 
{
    const struct servo_motor_pwm_api *api = (const struct servo_motor_pwm_api *)servo->api;
    if (api->offset_angle == NULL) 
    {
        return -ENOSYS;
    }
    return api->offset_angle(servo, offset_angle);
}

/**
 * @brief Set the angle of the servo motor.
 * @param servo servo motor device
 * @param angle motor angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static inline int servo_set_angle(const struct device *servo, float angle) 
{
    const struct servo_motor_pwm_api *api = (const struct servo_motor_pwm_api *)servo->api;
    if (api->set_angle == NULL) 
    {
        return -ENOSYS;
    }
    return api->set_angle(servo, angle);
}

/**
 * @brief Rotate the servo by a relative angle offset.
 * @param servo servo motor device
 * @param direction rotation direction (1 or -1 for clockwise or counterclockwise)
 * @param rotate_angle rotation angle in degrees
 * @retval int 0 on success, negative error code on failure.
 */
static inline int servo_rotate_angle(const struct device *servo, int8_t direction, float rotate_angle) 
{
    const struct servo_motor_pwm_api *api = (const struct servo_motor_pwm_api *)servo->api;
    if (api->rotate_angle == NULL) 
    {
        return -ENOSYS;
    }
    return api->rotate_angle(servo, direction, rotate_angle);
}


#ifdef __cplusplus
}
#endif

#endif  /* SERVO_MOTOR_PWM_H_ */
