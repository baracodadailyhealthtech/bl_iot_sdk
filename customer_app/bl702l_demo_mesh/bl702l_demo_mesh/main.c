/*
 * Copyright (c) 2016-2024 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "bl_wdt.h"
#include "btble_lib_api.h"
#include "bluetooth.h"
#include "hci_driver.h"
#include "conn.h"
#include "hci_core.h"
#if defined(CONFIG_BT_OAD_SERVER)
#include "oad_main.h"
#endif
#include "cli.h"
#if defined(CONFIG_BT_STACK_CLI)
#include"ble_cli_cmds.h"
#endif
#include "bt_log.h"

#include "include/pds_app.h"

#if defined(CFG_BLE_ENABLE)
extern int mesh_app_init(void);

#if defined(CONFIG_BT_OAD_SERVER)
bool app_check_oad(u32_t cur_file_ver, u32_t new_file_ver)
{
    //App layer decides whether to do oad according to file version
    /*if(new_file_ver > cur_file_ver)
        return true;
    else
        return false;*/
    return true;
}
#endif

void bt_enable_cb(int err)
{
    if (!err) {     
        #if defined(CONFIG_BT_OAD_SERVER)
        oad_service_enable(app_check_oad);
        #endif
        #ifdef CONFIG_BT_STACK_CLI 
        ble_cli_register();
        #endif
        mesh_app_init();
    }
}
void ble_init(void)
{
	 // Initialize BLE controller
    btble_controller_init(configMAX_PRIORITIES - 1);
    #if defined(CFG_PDS_ENABLE)
    btble_set_before_sleep_callback(pdsapp_before_sleep_callback);
    btble_set_after_sleep_callback(pdsapp_after_sleep_callback);
    btble_set_sleep_aborted_callback(pdsapp_sleep_aborted_callback);
    #endif
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
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
    ble_init();
    #if defined(CFG_PDS_ENABLE)
    pdsapp_init();
    #endif
}
