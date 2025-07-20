#ifndef  __PDS_APP_H__
#define  __PDS_APP_H__

#include "FreeRTOS.h"
#include "hal_pds.h"
#include "btble_pds.h"

int pdsapp_before_sleep_callback(void);
void pdsapp_after_sleep_callback(void);
void pdsapp_sleep_aborted_callback(void);
void pdsapp_init(void);

#endif // __PDS_APP_H__