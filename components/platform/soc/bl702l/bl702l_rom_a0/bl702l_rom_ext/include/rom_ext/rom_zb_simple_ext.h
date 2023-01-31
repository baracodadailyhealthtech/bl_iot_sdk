#ifndef __ROM_ZB_SIMPLE_EXT_H__
#define __ROM_ZB_SIMPLE_EXT_H__

#include "lmac154.h"
#include "zb_timer.h"

void rom_zb_simple_init(void);
void zb_pds_init(void);
//enable=1:device will enter into low power mode when it is idle;
//enable=0:device won't enter into low power mode when it
void zb_pds_enable(uint8_t enable);
void zbapp_action_before_sleep_callback(void);
void zbapp_action_after_wakeup_callback(void);
#endif
