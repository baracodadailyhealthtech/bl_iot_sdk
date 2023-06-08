#include "bl_ir.h"

uint16_t ir_tx[6] = {3, 2, 4, 3, 5, 4};

//interrupt callback after sending a pulse
void bl_ir_swm_tx_done_callback(void)
{
    printf("bl_ir_swm_tx_done_callback\r\n");
}

void ble_rc_ir_tx_demo(void)
{
    //gpio22: led0; gpio18: led1, not mounted on BL702L_DVK
    bl_ir_led_drv_cfg(1, 0);
    
    //frequency: 38.461KHz; duty cycle: 1/3
    bl_ir_swm_tx_cfg(38461, 0.3333);
    
    //send carriers
    bl_ir_swm_tx(ir_tx, 6);
    while(bl_ir_swm_tx_busy());
}
