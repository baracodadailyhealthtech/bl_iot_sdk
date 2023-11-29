/**
 * Copyright (c) 2016-2021 Bouffalolab Co., Ltd.
 *
 * Contact information:
 * web site:    https://www.bouffalolab.com/
 */

#include <stdio.h>
#include <cli.h>
#include <hosal_pwm.h>
#include <blog.h>
#if defined(CONF_USER_BL702)
#include "bl702_pwm.h"
#include "bl_irq.h"
#include "bl_gpio.h"
#endif

hosal_pwm_dev_t pwm;

void demo_hosal_pwm_init(void)
{
#if defined(CONF_USER_BL602) || defined(CONF_USER_BL702)
    /* pwm port and pin set  note: There is corresponding relationship between port and pin, for bl602/bl702, map is  port = pin%5 */
    pwm.port = 0;
    pwm.config.pin = 0;
#elif defined(CONF_USER_BL702L)
    /* for bl702l, pwm port can be 0 or 1
       if port = 0, pwm_v1 is used, which supports one channel, pin%5 == 0
       if port = 1, pwm_v2 is used, which supports four channels, pin%5 != 0 */
    pwm.port = 1;
    pwm.config.pin = 1;
#endif
    pwm.config.duty_cycle = 5000; //duty_cycle range is 0~10000 correspond to 0~100%
    pwm.config.freq = 1000;       //freq range is between 0~40MHZ,for more detail you can reference https://dev.bouffalolab.com/media/doc/602/open/reference_manual/zh/html/content/PWM.html
    /* init pwm with given settings */
    hosal_pwm_init(&pwm);
}

void demo_hosal_pwm_start(void)
{
    /* start pwm */
    hosal_pwm_start(&pwm);
}

void demo_hosal_pwm_change_param(void)
{
    /* change pwm param */
    hosal_pwm_config_t para;
    para.duty_cycle = 8000; //8000/10000=80%
    para.freq = 500;

    hosal_pwm_para_chg(&pwm, para);
}

void demo_hosal_pwm_stop(void)
{
    /* stop pwm */
    hosal_pwm_stop(&pwm);
    hosal_pwm_finalize(&pwm);
}

#if defined(CONF_USER_BL702)
void demo_pwm_sw_mode(uint32_t high_us, uint32_t low_us)
{
    demo_hosal_pwm_init();
    
    // enable sw mode, i.e. control pwm output level by software
    PWM_SW_Mode(pwm.port, ENABLE);
    
    while(1){
        // output high
        PWM_SW_Force_Value(pwm.port, 1);
        arch_delay_us(high_us);
        
        // output low
        PWM_SW_Force_Value(pwm.port, 0);
        arch_delay_us(low_us);
    }
}

void pwm_irq(void)
{
    static uint8_t init = 0;
    static uint8_t val = 0;
    const uint8_t pin = 1;
    
    // clear pwm interrupt status
    BL_WR_REG(PWM_BASE, PWM_INT_CONFIG, 1 << (PWM_INT_CLEAR_POS + pwm.port));
    
    if(!init){
        bl_gpio_enable_output(pin, 0, 0);
        init = 1;
    }
    
    bl_gpio_output_set(pin, val);
    val = !val;
}

void demo_pwm_interrupt(uint16_t period_cnt)
{
    uint32_t PWMx;
    uint32_t tmpVal;
    
    demo_hosal_pwm_init();
    
    // set interrupt period counter
    // when the cycle number of pwm output reaches this value, an interrupt will be generated
    PWMx = PWM_BASE + PWM_CHANNEL_OFFSET + pwm.port * 0x20;
    tmpVal = BL_RD_REG(PWMx, PWM_INTERRUPT);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, PWM_INT_PERIOD_CNT, period_cnt);
    BL_WR_REG(PWMx, PWM_INTERRUPT, tmpVal);
    
    // enable pwm interrupt
    PWM_IntMask(pwm.port, PWM_INT_PULSE_CNT, UNMASK);
    bl_irq_register(PWM_IRQn, pwm_irq);
    bl_irq_enable(PWM_IRQn);
    
    // start pwm
    demo_hosal_pwm_start();
}
#endif
