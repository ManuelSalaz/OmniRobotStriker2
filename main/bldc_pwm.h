#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/mcpwm_prelude.h"

#define MAP(val, in_min, in_max, out_min, out_max) \
    (((val) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

typedef struct {
    uint8_t rev_gpio_num;
    uint8_t pwm_gpio_num;
    uint32_t pwm_freq_hz;
    uint32_t pwm_duty_us;
    uint32_t max_cmp;
    uint16_t pwm_bottom_duty;
    uint16_t pwm_top_duty;
    uint16_t duty_cycle;
    float duty;
    bool is_calibrated;
    int group_id;
    uint32_t resolution_hz;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t cmp;
    mcpwm_cmpr_handle_t cmp_rev;
    mcpwm_gen_handle_t gen;
    mcpwm_gen_handle_t gen_rev;
} bldc_pwm_motor_t;

esp_err_t bldc_init(bldc_pwm_motor_t *motor, uint8_t pwm_gpio_num, uint8_t rev_gpio_num,
                    uint32_t pwm_freq_hz, uint32_t group_id, uint32_t resolution_hz,
                    uint16_t pwm_bottom_duty, uint16_t pwm_top_duty);

esp_err_t bldc_enable(bldc_pwm_motor_t *motor);
esp_err_t bldc_disable(bldc_pwm_motor_t *motor);
esp_err_t bldc_set_duty(bldc_pwm_motor_t *motor, int duty);
esp_err_t bldc_set_duty_motor(bldc_pwm_motor_t *motor, float duty);
esp_err_t bldc_calibrate(bldc_pwm_motor_t *motor, uint16_t bottom_duty, uint16_t top_duty);
