#include <bl702l_glb.h>
#include <bl702l_glb_gpio.h>
#include <bl702l_pwm.h>
#include "bl_pwm.h"

#define PWM_USE_ID PWM0_ID

int bl_pwm_init(uint8_t source, uint16_t div, uint16_t period)
{
    PWMx_CFG_Type pwmxCfg = {
        .clk = source,
        .stopMode = PWM_STOP_GRACEFUL,
        .clkDiv = div,
        .period = period,
        .intPulseCnt = 0,
        .extPol = PWM_BREAK_Polarity_LOW,
        .stpRept = DISABLE,
        .adcSrc = PWM_TRIGADC_SOURCE_NONE,
    };
    PWM_CHx_CFG_Type chxCfg = {
        .modP = PWM_MODE_DISABLE,
        .modN = PWM_MODE_DISABLE,
        .polP = PWM_POL_ACTIVE_HIGH,
        .polN = PWM_POL_ACTIVE_HIGH,
        .idlP = PWM_IDLE_STATE_INACTIVE,
        .idlN = PWM_IDLE_STATE_INACTIVE,
        .brkP = PWM_BREAK_STATE_INACTIVE,
        .brkN = PWM_BREAK_STATE_INACTIVE,
        .thresholdL = 0,
        .thresholdH = 0,
        .dtg = 0,
    };

    PWMx_Disable(PWM_USE_ID);
    PWMx_Init(PWM_USE_ID, &pwmxCfg);
    for (PWM_CHx_Type ch = PWM_CH0; ch < PWM_CHx_MAX; ch++) {
        PWM_Channelx_Init(PWM_USE_ID, ch, &chxCfg);
    }
    PWMx_Enable(PWM_USE_ID);

    return 0;
}

int bl_pwm_channel_init(uint8_t ch, uint8_t pin, uint8_t defaultLevel)
{
    GLB_GPIO_Cfg_Type gpioCfg;

    gpioCfg.gpioPin = pin;
    gpioCfg.gpioFun = GPIO_FUN_PWM;
    gpioCfg.gpioMode = GPIO_MODE_AF;
    gpioCfg.pullType = GPIO_PULL_NONE;
    gpioCfg.drive = 1;
    gpioCfg.smtCtrl = 1;
    GLB_GPIO_Init(&gpioCfg);

    PWM_Channelx_Positive_Pwm_Mode_Set(PWM_USE_ID, ch, PWM_MODE_DISABLE);
    if (defaultLevel) {
        PWM_Channelx_Positive_Polarity_Set(PWM_USE_ID, ch, PWM_POL_ACTIVE_LOW);
    } else {
        PWM_Channelx_Positive_Polarity_Set(PWM_USE_ID, ch, PWM_POL_ACTIVE_HIGH);
    }
    PWM_Channelx_Threshold_Set(PWM_USE_ID, ch, 0, 0);
    PWM_Channelx_Positive_Idle_State_Set(PWM_USE_ID, ch, PWM_IDLE_STATE_INACTIVE);
    PWM_Channelx_Positive_Pwm_Mode_Set(PWM_USE_ID, ch, PWM_MODE_ENABLE);

    return 0;
}

int bl_pwm_channel_duty_set(uint8_t ch, uint8_t duty)
{
    uint16_t period;
    uint16_t threshold;

    PWMx_Period_Get(PWM_USE_ID, &period);

    if (duty == 0) {
        PWM_Channelx_Threshold_Set(PWM_USE_ID, ch, 0, 0);
    } else if (duty >= 100) {
        PWM_Channelx_Threshold_Set(PWM_USE_ID, ch, 0, period);
    } else {
        threshold = (uint16_t)(((uint32_t)period * (uint32_t)duty + 50) / 100);
        PWM_Channelx_Threshold_Set(PWM_USE_ID, ch, 0, threshold);
    }

    return 0;
}
