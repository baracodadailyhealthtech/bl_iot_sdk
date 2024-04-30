/*
 * Copyright (c) 2016-2024 Bouffalolab.
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
#if !defined(IR_OUTPUT_INVERSE)
        .polP = PWM_POL_ACTIVE_LOW,
        .polN = PWM_POL_ACTIVE_LOW,
#else
        .polP = PWM_POL_ACTIVE_HIGH,
        .polN = PWM_POL_ACTIVE_HIGH,
#endif
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

int bl_pwm_ir_tx(uint16_t data[], uint32_t len)
{
    int mstatus;
    uint32_t i;
    uint32_t tmpVal;

    mstatus = bl_irq_save();

    for(i=0; i<len; i++){
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

int bl_pwm_ir_tx_ex(uint32_t data[], uint32_t len)
{
    int mstatus;
    uint32_t i;
    uint32_t tmpVal;

    mstatus = bl_irq_save();

    for(i=0; i<len; i++){
        tmpVal = BL_RD_REG(PWM_BASE, PWM_MC0_PERIOD);
        tmpVal = BL_SET_REG_BITS_VAL(tmpVal, PWM_INT_PERIOD_CNT, (uint16_t)data[i]);
        BL_WR_REG(PWM_BASE, PWM_MC0_PERIOD, tmpVal);

        if((data[i] >> 31) == 1){
            pwm_mc_set_threshold(pwm_ch, pwm_threshold);
        }else{
            pwm_mc_set_threshold(pwm_ch, 0);
        }

        pwm_mc_start(pwm_ch);
    }

    bl_irq_restore(mstatus);

    return 0;
}


void bl_pwm_ir_nec_tx_init(uint8_t pin)
{
    bl_pwm_ir_tx_cfg(37700, 0.3333);
    bl_pwm_ir_tx_pin_cfg(pin);
}

int bl_pwm_ir_nec_tx(uint8_t addr, uint8_t cmd)
{
    uint32_t data = addr | ((uint8_t)(~addr) << 8) | (cmd << 16) | ((uint8_t)(~cmd) << 24);
    uint16_t ir_tx[2 + 64 + 2];

    //printf("addr: 0x%02X, cmd: 0x%02X, data: 0x%08lX\r\n", addr, cmd, data);

    // head
    ir_tx[0] = 340;  // 9ms
    ir_tx[1] = 170;  // 4.5ms

    // data (lsb first)
    for(int i=0; i<32; i++){
        if(data & (0x1 << i)){
            ir_tx[2 + 2*i] = 21;      // 560us
            ir_tx[2 + 2*i + 1] = 64;  // 1690us
        }else{
            ir_tx[2 + 2*i] = 21;      // 560us
            ir_tx[2 + 2*i + 1] = 21;  // 560us
        }
    }

    // tail
    ir_tx[2 + 64] = 21;  // 560us
    ir_tx[2 + 65] = 21;  // 560us

    return bl_pwm_ir_tx(ir_tx, sizeof(ir_tx)/sizeof(ir_tx[0]));
}

int bl_pwm_ir_nec_tx_repeat(void)
{
    uint16_t ir_tx[3];

    ir_tx[0] = 340;  // 9ms
    ir_tx[1] = 85;   // 2.25ms
    ir_tx[2] = 21;   // 560us

    return bl_pwm_ir_tx(ir_tx, sizeof(ir_tx)/sizeof(ir_tx[0]));
}


void bl_pwm_ir_rc5_tx_init(uint8_t pin)
{
    bl_pwm_ir_tx_cfg(36000, 0.3333);
    bl_pwm_ir_tx_pin_cfg(pin);
}

int bl_pwm_ir_rc5_tx(uint8_t t, uint8_t addr, uint8_t cmd)
{
    uint32_t data = ((t & 0x1) << 11) | ((addr & 0x1F) << 6) | (cmd & 0x3F);
    uint32_t ir_tx[28];

    //printf("t: 0x%02X, addr: 0x%02X, cmd: 0x%02X, data: 0x%08lX\r\n", t, addr, cmd, data);

    // S1 (logical 1)
    ir_tx[0] = 32;
    ir_tx[1] = 32 | 0x80000000;

    // S2 (logical 1)
    ir_tx[2] = 32;
    ir_tx[3] = 32 | 0x80000000;

    // T + Address + Command (msb first)
    for(int i=0; i<12; i++){
        if(data & (0x1 << (11 - i))){
            ir_tx[4 + 2*i] = 32;
            ir_tx[4 + 2*i + 1] = 32 | 0x80000000;
        }else{
            ir_tx[4 + 2*i] = 32 | 0x80000000;
            ir_tx[4 + 2*i + 1] = 32;
        }
    }

    return bl_pwm_ir_tx_ex(ir_tx, sizeof(ir_tx)/sizeof(ir_tx[0]));
}


#if 0
void bl_pwm_ir_test(void)
{
    uint16_t data[] = {2, 2, 4, 2, 6};

    bl_pwm_ir_tx_cfg(37700, 0.3333);
    bl_pwm_ir_tx_pin_cfg(22);

    while(1){
        bl_pwm_ir_tx(data, sizeof(data)/sizeof(data[0]));
        arch_delay_ms(500);
    }
}

void bl_pwm_ir_nec_test(void)
{
    bl_pwm_ir_nec_tx_init(22);

    while(1){
        bl_pwm_ir_nec_tx(0x56, 0x78);
        arch_delay_ms(500);
    }
}

void bl_pwm_ir_rc5_test(void)
{
    bl_pwm_ir_rc5_tx_init(22);

    while(1){
        bl_pwm_ir_rc5_tx(0, 0x12, 0x34);
        arch_delay_ms(500);
    }
}
#endif
