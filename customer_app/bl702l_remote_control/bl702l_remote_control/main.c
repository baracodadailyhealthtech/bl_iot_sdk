#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rom_hal_ext.h"
#include "cli.h"

extern uint8_t pds_start;
static void cmd_start_pds(char *buf, int len, int argc, char **argv)
{
    pds_start = 1;  
}

const struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
    {"pds_start", "enable pds", cmd_start_pds},
};

void main(void)
{
    #if defined(CFG_BLE_PDS)
    bl_pds_init();
    #endif
    extern void ble_stack_start(void);
    ble_stack_start();;
}
