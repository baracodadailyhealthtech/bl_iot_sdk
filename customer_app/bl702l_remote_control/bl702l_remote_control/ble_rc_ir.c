#include "bl_ir.h"

#define PW_UNIT     1125  // 562.5us

uint32_t ir_tx[3] = {0x80000064, 0x00000032, 0x80000014};
uint16_t ir_len = 3;
uint16_t ir_idx = 0;

//interrupt callback after sending a pulse
void bl_ir_swm_tx_done_callback(void)
{
    ir_idx++;
    if(ir_idx < ir_len){
        //send next pulse
        bl_ir_swm_tx(PW_UNIT, ir_tx[ir_idx] & 0x7FFFFFFF, ir_tx[ir_idx] >> 31);
    }else{
        //send done
        ir_idx = 0;
    }
}

void ble_rc_ir_tx_demo(void)
{
    //gpio22: led0; gpio18: led1, not mounted on BL702L_DVK
    bl_ir_led_drv_cfg(1, 0);
    
    //frequency: 38.461KHz; duty cycle: 1/3
    bl_ir_swm_tx_cfg(38461, 0.3333);
    
    //send first pulse
    bl_ir_swm_tx(PW_UNIT, ir_tx[0] & 0x7FFFFFFF, ir_tx[0] >> 31);
}
