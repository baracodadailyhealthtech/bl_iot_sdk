/*
 * Copyright (c) 2016-2023 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __BL_PWM_H__
#define __BL_PWM_H__

#include <stdint.h>

typedef enum {
    BL_PWM0 = 0,
    BL_PWM1 = 1,
}bl_pwm_t;

typedef enum {
    BL_PWM0_CH0 = 0,
    BL_PWM1_CH0 = 1,
    BL_PWM1_CH1 = 2,
    BL_PWM1_CH2 = 3,
    BL_PWM1_CH3 = 4,
}bl_pwm_ch_t;

int32_t bl_pwm_port_init(bl_pwm_t id, uint32_t freq);
int32_t bl_pwm_gpio_init(bl_pwm_ch_t ch, uint8_t pin);  // ch == pin % 5
int32_t bl_pwm_start(bl_pwm_ch_t ch);
int32_t bl_pwm_stop(bl_pwm_ch_t ch);
int32_t bl_pwm_set_duty(bl_pwm_ch_t ch, float duty);
// set duty cycle according to threshold1 and return actual threshold1 & threshold2
// if hardware counter is within [threshold1, threshold2], the pwm output is high; refer to reference manual for details
/*
    uint16_t threshold1, threshold2;
    threshold1 = 0;
    bl_pwm_set_duty_ex(BL_PWM1_CH0, 20, &threshold1, &threshold2);  // PWM1_CH0: duty = 20%, will output high right away after start because threshold1 is 0
    threshold1 = threshold2;
    bl_pwm_set_duty_ex(BL_PWM1_CH1, 30, &threshold1, &threshold2);  // PWM1_CH1: duty = 30%, will output high once PWM1_CH0 output low
*/
int32_t bl_pwm_set_duty_ex(bl_pwm_ch_t ch, float duty, uint16_t *threshold1, uint16_t *threshold2);
int32_t bl_pwm_get_duty(bl_pwm_ch_t ch, float *p_duty);

#endif
