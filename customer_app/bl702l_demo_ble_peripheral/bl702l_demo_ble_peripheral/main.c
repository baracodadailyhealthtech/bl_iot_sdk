#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "bl_wdt.h"
#include "cli.h"
#include "btble_lib_api.h"
#include "ble_app.h"
#include "pds_app.h"

static void cmd_start_pds(char *buf, int len, int argc, char **argv)
{
    btble_pds_enable(1);  
}

static void cmd_wdt_set(char *buf, int len, int argc, char **argv)
{
    if(argc != 2){
        printf("Number of parameters error.\r\n");
        return;
    }

    if(strcmp(argv[1], "enable") == 0){
        bl_wdt_init(4000);
    }else if(strcmp(argv[1], "disable") == 0){
        bl_wdt_disable();
    }else{
        printf("Second parameter error.\r\n");
    }
}

const struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
    {"pds_start", "enable pds", cmd_start_pds},
    {"wdt_set", "enable or disable wdt", cmd_wdt_set},
};

void _dump_lib_info(void)
{
    puts("BTBLE Controller LIB Version: ");
    puts(btble_controller_get_lib_ver());
    puts("\r\n");
}

void main(void)
{ 
    ble_stack_start();
    #if defined(CFG_PDS_ENABLE)
    pdsapp_init();
    #endif
}

