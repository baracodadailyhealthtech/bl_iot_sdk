#include <stdio.h>
#include <stdint.h>
#include <string.h>
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
    ble_stack_start();
    #if defined(CFG_PDS_ENABLE)
    pdsapp_init();
    #endif
}

