/*
 * Copyright (c) 2016-2023 Bouffalolab.
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
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <vfs.h>
#include <device/vfs_uart.h>
#include <aos/kernel.h>
#include <aos/yloop.h>
#include <event_device.h>
#include <cli.h>

#include <bl_sys.h>
#include <bl_chip.h>
#include <bl_irq.h>
#include <bl_sec.h>
#include <bl_rtc.h>
#include <bl_uart.h>
#include <bl_gpio.h>
#include <bl_flash.h>
#include <bl_timer.h>
#include <bl_wdt.h>
#include <hal_board.h>
#include <hal_gpio.h>
#include <hal_button.h>

#if defined(CFG_BLE_ENABLE)
#include "bluetooth.h"
#include "ble_cli_cmds.h"
#include "hci_driver.h"
#include "hci_core.h"
#include "btble_lib_api.h"
#endif

#if defined(CFG_ZIGBEE_ENABLE)
#include "zb_common.h"
#if defined(CFG_ZIGBEE_CLI)
#include "zb_stack_cli.h"
#endif
#include "zigbee_app.h"
//#include "zb_bdb.h"
#endif

void _cli_init(int fd_console)
{

}

void _dump_lib_info(void)
{

}

static void cmd_uart_boot(char *buf, int len, int argc, char **argv)
{
    HBN_Set_User_Boot_Config(1);

    bl_sys_reset_system();
}

#if defined(CFG_BLE_PDS)
extern uint8_t pds_start;
static void cmd_start_pds(char *buf, int len, int argc, char **argv)
{
    pds_start = 1;  
}
#endif

const static struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
    {"uart_boot", "enter uart boot mode", cmd_uart_boot},
    #if defined(CFG_BLE_PDS)
    {"pds_start", "enable pds", cmd_start_pds},
    #endif
};

#if defined(CFG_ZIGBEE_ENABLE)
void zigbee_init(void)
{
    zbRet_t status;
    status = zb_stackInit();
    if (status != ZB_SUCC)
    {
        printf("BL Zbstack Init fail : 0x%08x\r\n", status);
        //ASSERT(false);
    }
    else
    {
        printf("BL Zbstack Init Success\r\n");
    }
    #if defined(CFG_ZIGBEE_CLI)
    zb_cli_register();
    #endif
    register_zb_cb();
    
    zb_app_startup();

    //zb_bdb_init();
}
#endif

void event_cb_key_event(input_event_t *event, void *private_data)
{
    switch (event->code) {
        case KEY_1:
        {
            printf("[KEY_1] [EVT] INIT DONE %lld\r\n", aos_now_ms());
            printf("short press \r\n");
        }
        break;
        case KEY_2:
        {
            printf("[KEY_2] [EVT] INIT DONE %lld\r\n", aos_now_ms());
            printf("long press \r\n");
        }
        break;
        case KEY_3:
        {
            printf("[KEY_3] [EVT] INIT DONE %lld\r\n", aos_now_ms());
            printf("longlong press \r\n");
        }
        break;
        default:
        {
            printf("[KEY] [EVT] Unknown code %u, %lld\r\n", event->code, aos_now_ms());
            /*nothing*/
        }
    }
}

#if defined(CFG_BLE_ENABLE)
void bt_enable_cb(int err)
{
    if (!err) {
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);

#ifdef CONFIG_BT_STACK_CLI 
        ble_cli_register();
#endif
    }
}

void ble_stack_start(void)
{       
     // Initialize BLE controller
    btble_controller_init(configMAX_PRIORITIES - 1);
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
}
#endif

static void system_init(void)
{
    #if defined(CFG_BLE_PDS)
    extern void ble_pds_init(void);
    ble_pds_init();
    #endif
    #if defined(CFG_ZIGBEE_PDS)
    extern void zb_pds_init(void);
    zb_pds_init();
    #if defined(CFG_ZB_SIMPLE_MODE_ENABLE)
    extern void rom_zb_simple_init(void);
    rom_zb_simple_init();
    #endif
    #endif
}

static void system_thread_init(void)
{
    uint32_t fdt = 0, offset = 0;

    if (0 == hal_board_get_dts_addr("gpio", &fdt, &offset)) {
        hal_gpio_init_from_dts(fdt, offset);
        fdt_button_module_init((const void *)fdt, (int)offset);
    }

    aos_register_event_filter(EV_KEY, event_cb_key_event, NULL);

    #if defined(CFG_BLE_ENABLE)
    ble_stack_start();
    #endif
    #if defined(CFG_ZIGBEE_ENABLE)
    #if defined(CFG_ZIGBEE_SLEEPY_END_DEVICE_STARTUP) && (CFG_PDS_LEVEL == 31)
    zb_disableFlashCache();
    #endif
    zigbee_init();
    #endif
}

void main(void)
{
    system_init();
    system_thread_init();
}
