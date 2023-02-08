#ifndef __BL_PWM_H__
#define __BL_PWM_H__

#include <stdint.h>

// source: 0-xclk, 1-bclk, 2-f32k
// div: 1 ~ 65535
// period: 1 ~ 65535
int bl_pwm_init(uint8_t source, uint16_t div, uint16_t period);

// ch: 0 ~ 3
// pin: pin % 5 = ch + 1
int bl_pwm_channel_init(uint8_t ch, uint8_t pin, uint8_t defaultLevel);

// duty: 0 ~ 100
int bl_pwm_channel_duty_set(uint8_t ch, uint8_t duty);

#endif
