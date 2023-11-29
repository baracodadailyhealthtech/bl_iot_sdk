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
#include "bl_pwm_ir.h"
#include "bl_irq.h"


static uint8_t pwm_ch = 0;
static uint16_t pwm_threshold = 0;


static void pwm_mc_init(uint16_t div, uint16_t period)
{
    PWMx_CFG_Type pwmxCfg = {
        .clk = PWM_CLK_BCLK,
        .stopMode = PWM_STOP_GRACEFUL,
        .clkDiv = div,
        .period = period,
        .intPulseCnt = 0,
        .extPol = PWM_BREAK_Polarity_LOW,
        .stpRept = ENABLE,
        .adcSrc = PWM_TRIGADC_SOURCE_NONE,
    };

    PWM_CHx_CFG_Type chxCfg = {
        .modP = PWM_MODE_DISABLE,
        .modN = PWM_MODE_DISABLE,
        .polP = PWM_POL_ACTIVE_LOW,
        .polN = PWM_POL_ACTIVE_LOW,
        .idlP = PWM_IDLE_STATE_INACTIVE,
        .idlN = PWM_IDLE_STATE_INACTIVE,
        .brkP = PWM_BREAK_STATE_INACTIVE,
        .brkN = PWM_BREAK_STATE_INACTIVE,
        .thresholdL = 0,
        .thresholdH = 0,
        .dtg = 0,
    };

    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_PWM);

    PWMx_Disable(PWM0_ID);
    PWMx_Init(PWM0_ID, &pwmxCfg);
    for(PWM_CHx_Type ch = PWM_CH0; ch < PWM_CHx_MAX; ch++){
        PWM_Channelx_Init(PWM0_ID, ch, &chxCfg);
    }
    PWM_Int_Mask(PWM0_ID, PWM_INT_REPT, UNMASK);
    PWMx_Enable(PWM0_ID);
}

static void pwm_mc_set_threshold(uint8_t ch, uint16_t threshold)
{
    PWM_Channelx_Threshold_Set(PWM0_ID, ch, 1, threshold + 1);
}

static void pwm_mc_start(uint8_t ch)
{
    PWM_Int_Clear(PWM0_ID, PWM_INT_REPT);
    PWM_Channelx_Positive_Pwm_Mode_Set(PWM0_ID, ch, PWM_MODE_ENABLE);
    while(PWM_Int_Status_Get(PWM0_ID, PWM_INT_REPT) == 0);
}


void bl_pwm_ir_tx_cfg(float freq_hz, float duty_cycle)
{
    uint32_t div = 1;
    uint32_t period = SystemCoreClockGet()/(GLB_Get_BCLK_Div()+1)/(uint32_t)freq_hz;

    while(period >= 65536){
        div <<= 1;
        period >>= 1;
    }

    pwm_mc_init((uint16_t)div, (uint16_t)period);

    pwm_threshold = (uint16_t)(period * duty_cycle);
}

void bl_pwm_ir_tx_pin_cfg(uint8_t pin)
{
    GLB_GPIO_Cfg_Type cfg;

    cfg.drive = 3;
    cfg.smtCtrl = 1;
    cfg.gpioMode = GPIO_MODE_OUTPUT;
    cfg.pullType = GPIO_PULL_NONE;
    cfg.gpioPin = pin;
    cfg.gpioFun = 8;

    GLB_GPIO_Init(&cfg);

    pwm_ch = (pin - 1) % 5;

    pwm_mc_set_threshold(pwm_ch, 0);
    pwm_mc_start(pwm_ch);
}

int bl_pwm_ir_tx(uint16_t data[], uint8_t len)
{
    int mstatus;
    uint32_t tmpVal;

    mstatus = bl_irq_save();

    for(int i=0; i<len; i++){
        tmpVal = BL_RD_REG(PWM_BASE, PWM_MC0_PERIOD);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, PWM_INT_PERIOD_CNT, data[i]);
        BL_WR_REG(PWM_BASE, PWM_MC0_PERIOD, tmpVal);

        if(i % 2 == 0){
            pwm_mc_set_threshold(pwm_ch, pwm_threshold);
        }else{
            pwm_mc_set_threshold(pwm_ch, 0);
        }

        pwm_mc_start(pwm_ch);
    }

    bl_irq_restore(mstatus);

    return 0;
}
