#if defined(CFG_BLE_PDS)
#include "rom_hal_ext.h"
#include "rom_freertos_ext.h"
#include "btble_lib_api.h"

void ble_pds_init(void)
{
    bl_pds_init();
    btble_controller_sleep_init();
}
#endif//(CFG_BLE_PDS)