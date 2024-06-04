#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cli.h"
#include "bl_kys.h"
#include "bl_adc.h"
#include "btble_lib_api.h"
#include "ble_rc_app.h"
#include "ble_rc_ir.h"
#if defined(CFG_PDS_ENABLE)
#include <FreeRTOS.h>
#include "btble_pds.h"
#endif

#if defined(CFG_PDS_ENABLE)
#if !defined(CFG_USE_ROM_CODE) || defined(CFG_BUILD_FREERTOS)
void vApplicationSleep(TickType_t xExpectedIdleTime)
{
    btble_vApplicationSleepExt(xExpectedIdleTime);
}
#endif
#endif

void _dump_lib_info(void)
{
    puts("BTBLE Controller LIB Version: ");
    puts(btble_controller_get_lib_ver());
    puts("\r\n");
}

void main(void)
{
    #if defined(CFG_PDS_ENABLE)
    #if defined(CFG_USE_ROM_CODE) && !defined(CFG_BUILD_FREERTOS)
    vApplicationSleep = btble_vApplicationSleepExt;
    #endif
    btble_pds_init(NULL);
    #endif
    ble_rc_kys_init();
    ble_rc_ir_tx_init();
    ble_stack_start();
}
