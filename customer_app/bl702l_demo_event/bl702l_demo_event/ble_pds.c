#if defined(CFG_BLE_PDS)
#include "rom_hal_ext.h"
#include "rom_freertos_ext.h"

void ble_pds_init(void)
{
    bl_pds_init();
}
#endif//(CFG_BLE_PDS)