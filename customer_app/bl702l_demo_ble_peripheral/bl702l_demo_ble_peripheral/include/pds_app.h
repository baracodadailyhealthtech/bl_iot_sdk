#ifndef  __PDS_APP_H__
#define  __PDS_APP_H__

#include "rom_freertos_ext.h"
#include "rom_hal_ext.h"
#include "rom_btble_ext.h"

int pdsapp_before_sleep_callback(void);
void pdsapp_after_sleep_callback(void);
void pdsapp_sleep_aborted_callback(void);
void pdsapp_init(void);

#endif // __PDS_APP_H__