#ifndef __BLE_RC_IR_H__
#define __BLE_RC_IR_H__

#include <stdint.h>

void ble_rc_ir_tx_init(void);
void ble_rc_ir_tx_demo(void);
void ble_rc_ir_tx_repeat_demo(void);

void ble_rc_ir_timer_start(uint32_t delay_us, void (*cb)(void));
void ble_rc_ir_timer_stop(void);
int ble_rc_ir_is_busy(void);
void ble_rc_ir_tx_demo_with_timer(void);
void ble_rc_ir_tx_repeat_demo_with_timer(void);

#endif //__BLE_RC_IR_H__
