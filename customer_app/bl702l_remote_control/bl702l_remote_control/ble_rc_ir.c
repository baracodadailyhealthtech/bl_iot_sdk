#include "bl_ir.h"
#include "bl_pwm_ir.h"

#define USE_PWM_IR

#if !defined(USE_PWM_IR)
//interrupt callback after sending a pulse
void bl_ir_swm_tx_done_callback(void)
{
    printf("bl_ir_swm_tx_done_callback\r\n");
}
#endif

void ble_rc_ir_tx_init(void)
{
#if defined(USE_PWM_IR)
    //frequency: 37.7KHz; duty cycle: 1/3
    bl_pwm_ir_tx_cfg(37700, 0.3333);
    
    //ir output on gpio22
    bl_pwm_ir_tx_pin_cfg(22);
#else
    //frequency: 37.7KHz; duty cycle: 1/3
    bl_ir_swm_tx_cfg(37700, 0.3333);
    
    //gpio22: led0; gpio18: led1, not mounted on BL702L_DVK
    bl_ir_led_drv_cfg(1, 0);
#endif
}

void ble_rc_ir_tx_demo(void)
{
    uint16_t ir_tx[6] = {3, 2, 4, 2, 5};
    
#if defined(USE_PWM_IR)
    bl_pwm_ir_tx(ir_tx, sizeof(ir_tx)/2);
#else
    bl_ir_swm_tx(ir_tx, sizeof(ir_tx)/2);
    while(bl_ir_swm_tx_busy());
#endif
}

void ble_rc_ir_tx_repeat(void)
{
#if defined(USE_PWM_IR)
    uint16_t ir_tx[] = {340, 85, 21};
    bl_pwm_ir_tx(ir_tx, sizeof(ir_tx)/2);
#endif
}
