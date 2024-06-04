#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "bl_wdt.h"
#include "cli.h"
#include "btble_lib_api.h"
#include "ble_ota_app.h"
#include "ble_ota_config.h"

#if (ROLE_SELECT == 0)
static void cmd_slave_ota_start(char *buf, int len, int argc, char **argv)
{
    ble_adv_start();
}
#endif

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

const static struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
	#if (ROLE_SELECT == 0)
	{"ota_start", "slave ota_start", cmd_slave_ota_start},
	#endif
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
}
