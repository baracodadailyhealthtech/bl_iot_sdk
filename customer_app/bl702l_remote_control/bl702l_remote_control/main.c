#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rom_hal_ext.h"
#include "cli.h"
#include "bl_kys.h"
#include "ble_rc_app.h"

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
    //gpio0 cannot be used in keyscan.
    //row number needs to be equal to col number
    uint8_t row_pins[] = {31, /*10,*/25, 24, 23};
    uint8_t col_pins[] = {9, /*0,*/ 28, 27, 26};
    bl_kys_init(sizeof(row_pins), sizeof(col_pins), row_pins, col_pins);
    ble_stack_start();
}
