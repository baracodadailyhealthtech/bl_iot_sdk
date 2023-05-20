#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rom_hal_ext.h"
#include "rom_btble_ext.h"
#include "cli.h"
#include "ble_app.h"
#include "pds_app.h"

static void cmd_start_pds(char *buf, int len, int argc, char **argv)
{
    btble_pds_enable(1);  
}

const struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
    {"pds_start", "enable pds", cmd_start_pds},
};

void main(void)
{
    #if defined(CFG_BLE_PDS)
    pdsapp_init();
    #endif
 
    ble_stack_start();
}

