#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cli.h"
#include "ble_ota_app.h"
#include "ble_ota_config.h"

#if (ROLE_SELECT == 0)
static void cmd_slave_ota_start(char *buf, int len, int argc, char **argv)
{
    ble_adv_start();
}
#endif

const static struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
	#if (ROLE_SELECT == 0)
	{"ota_start", "slave ota_start", cmd_slave_ota_start},
	#endif
};

void main(void)
{
    ble_stack_start();
}
