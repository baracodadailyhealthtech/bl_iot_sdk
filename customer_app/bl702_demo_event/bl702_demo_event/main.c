/*
 * Copyright (c) 2016-2025 Bouffalolab.
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
#include <bl_wireless.h>
#include <bl_irq.h>
#include <bl_sec.h>
#include <bl_rtc.h>
#include <bl_uart.h>
#include <bl_gpio.h>
#include <bl_flash.h>
#include <bl_hbn.h>
#include <bl_timer.h>
#include <bl_wdt.h>
#include <hal_boot2.h>
#include <hal_board.h>
#include <hosal_uart.h>
#include <hosal_gpio.h>
#include <hal_gpio.h>
#include <hal_button.h>
#include <hal_pds.h>
#include <hal_tcal.h>

#ifdef CFG_ETHERNET_ENABLE
#include <lwip/netif.h>
#include <lwip/etharp.h>
#include <lwip/udp.h>
#include <lwip/ip.h>
#include <lwip/init.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/dhcp.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>
#include <lwip/netifapi.h>

#include <bl_sys_ota.h>
#include <bl_emac.h>
#include <bl702_glb.h>
#include <bl702_common.h>
#include <bflb_platform.h>
#include <eth_bd.h>
#include <netutils/netutils.h>

#include <netif/ethernet.h>
#endif /* CFG_ETHERNET_ENABLE */
#include <easyflash.h>
#include <libfdt.h>
#include <utils_log.h>
#include <blog.h>

#ifdef EASYFLASH_ENABLE
#include <easyflash.h>
#endif
#include <utils_string.h>
#if defined(CONFIG_AUTO_PTS)
#include "bttester.h"
#include "autopts_uart.h"
#endif

#if defined(CFG_BLE_ENABLE)
#include "bluetooth.h"
#include "ble_cli_cmds.h"
#include <hci_driver.h>
#include "ble_lib_api.h"

#if defined(CONFIG_BLE_TP_SERVER)
#include "ble_tp_svc.h"
#endif

#if defined(CONFIG_BT_MESH)
#include "mesh_cli_cmds.h"
#endif
#endif

#if defined(CFG_ZIGBEE_ENABLE)
#include "zb_common.h"
#if defined(CFG_ZIGBEE_CLI)
#include "zb_stack_cli.h"
#endif
#include "zigbee_app.h"
#endif
#if defined(CONFIG_ZIGBEE_PROV)
#include "blsync_ble_app.h"
#endif
#if defined(CFG_USE_PSRAM)
#include "bl_psram.h"
#endif /* CFG_USE_PSRAM */

#ifdef CFG_ETHERNET_ENABLE
//extern err_t ethernetif_init(struct netif *netif);
extern err_t eth_init(struct netif *netif);
static void netif_status_callback(struct netif *netif)
{
    if (netif->flags & NETIF_FLAG_UP) {
#if LWIP_IPV4
        if(!ip4_addr_isany(netif_ip4_addr(netif))){
            char addr[INET_ADDRSTRLEN];
            const ip4_addr_t* ipv4addr = netif_ip4_addr(netif);
            inet_ntop(AF_INET, ipv4addr, addr, sizeof(addr));
            printf("IP: %s\r\n", addr);

            const ip4_addr_t* ipv4mask = netif_ip4_netmask(netif);
            inet_ntop(AF_INET, ipv4mask, addr, sizeof(addr));
            printf("MASK: %s\r\n", addr);

            const ip4_addr_t* ipv4gw = netif_ip4_gw(netif);
            inet_ntop(AF_INET, ipv4gw, addr, sizeof(addr));
            printf("Gateway: %s\r\n", addr);
        }
#endif

#if LWIP_IPV6
        for (uint32_t i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i ++ ) {
          if (!ip6_addr_isany(netif_ip6_addr(netif, i))
                && ip6_addr_ispreferred(netif_ip6_addr_state(netif, i))
                ) {
            const ip6_addr_t* ip6addr = netif_ip6_addr(netif, i);
            char addr[INET6_ADDRSTRLEN];

            if (ip6_addr_isany(ip6addr)) {
                continue;
            }
            inet_ntop(AF_INET6, ip6addr, addr, sizeof(addr));
            if(ip6_addr_islinklocal(netif_ip6_addr(netif, i))){
                printf("LOCAL IP6 addr %s\r\n", addr);
            }
            else{
                printf("GLOBAL IP6 addr %s\r\n", addr);
            }
          }
        }
#endif
    }
    else {
        printf("interface is down status.\n");
    }
}
static int app_eth_callback(eth_link_state val)
{
    switch(val){
    case ETH_INIT_STEP_LINKUP:{

    }break;
    case ETH_INIT_STEP_READY:{
        netif_set_default(&eth_mac);
        netif_set_up(&eth_mac);
        dhcp_start(&eth_mac);
        printf("start dhcp....\r\n");
    }break;
    case ETH_INIT_STEP_LINKDOWN:{

    }break;
    }
    return 0;
}

void lwip_init_netif(void)
{
    ip_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr, 0,0,0,0);
    IP4_ADDR(&netmask, 0,0,0,0);
    IP4_ADDR(&gw, 0,0,0,0);
    netif_add(&eth_mac, &ipaddr, &netmask, &gw, NULL, eth_init, ethernet_input);

    ethernet_init(app_eth_callback);
    /* Set callback to be called when interface is brought up/down or address is changed while up */
    netif_set_status_callback(&eth_mac, netif_status_callback);
}
#endif /* CFG_ETHERNET_ENABLE */

void vApplicationMallocFailedHook(void)
{
    printf("Memory Allocate Failed. Current left size is %d bytes\r\n"
#if defined(CFG_USE_PSRAM)
        "Current psram left size is %d bytes\r\n"
#endif /*CFG_USE_PSRAM*/
        ,xPortGetFreeHeapSize()
#if defined(CFG_USE_PSRAM)
        ,xPortGetFreeHeapSizePsram()
#endif /*CFG_USE_PSRAM*/
    );
    while (1) {
        /*empty here*/
    }
}

void vApplicationIdleHook(void)
{
    bl_wdt_feed();
    __asm volatile("wfi");
}

#if ( configUSE_TICK_HOOK != 0 )
void vApplicationTickHook( void )
{
#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_monitor(void);
    usb_cdc_monitor();
#endif
#if defined(CFG_ZIGBEE_ENABLE)
	extern void ZB_MONITOR(void);
	ZB_MONITOR();
#endif
}
#endif

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    //static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
    static StackType_t uxIdleTaskStack[256];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    //*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE; 
    *pulIdleTaskStackSize = 256;//size 256 words is For ble pds mode, otherwise stack overflow of idle task will happen.
}

void _cli_init(int fd_console)
{
#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_start(int fd_console);
    usb_cdc_start(fd_console);
#endif

    /*Put CLI which needs to be init here*/
#if defined(CFG_EFLASH_LOADER_ENABLE)
    extern int helper_eflash_loader_cli_init(void);
    helper_eflash_loader_cli_init();
#endif

#if defined(CFG_RFPHY_CLI_ENABLE)
    extern int helper_rfphy_cli_init(void);
    helper_rfphy_cli_init();
#endif
#ifdef CFG_ETHERNET_ENABLE
    /*Put CLI which needs to be init here*/
    network_netutils_iperf_cli_register();
    network_netutils_ping_cli_register();
    bl_sys_ota_cli_init();
#endif /* CFG_ETHERNET_ENABLE */
}

#if defined(CFG_BLE_ENABLE)
void ble_init(void)
{
    extern void ble_stack_start(void);
    ble_stack_start();
}
#endif

#if defined(CFG_ZIGBEE_ENABLE)
void zigbee_init(void)
{
    zbRet_t status;
    status = zb_stackInit();
    if (status != ZB_SUCC)
    {
        printf("BL Zbstack Init fail : 0x%08x\r\n", status);
    }
    else
    {
        printf("BL Zbstack Init Success\r\n");
    }
    #if defined(CFG_ZIGBEE_CLI)
    zb_cli_register();
    #endif

    zb_app_startup();
}
#endif

void _dump_lib_info(void)
{
#if defined(CFG_BLE_ENABLE)
    puts("BLE Controller LIB Version: ");
    puts(ble_controller_get_lib_ver());
    puts("\r\n");
#endif
}

static void system_init(void)
{
#if defined(CFG_WATCHDOG_ENABLE)
    bl_wdt_init(4000);
#endif
}

static void system_thread_init()
{
#ifndef CFG_ETHERNET_ENABLE
    uint32_t fdt = 0, offset = 0;

    if (0 == hal_board_get_dts_addr("gpio", &fdt, &offset)) {
        hal_gpio_init_from_dts(fdt, offset);
        fdt_button_module_init((const void *)fdt, (int)offset);
    }
#endif /* CFG_ETHERNET_ENABLE */

#if defined(CFG_BLE_ENABLE)
    #if defined(CONFIG_AUTO_PTS)
    pts_uart_init(1,115200,8,1,0,0);
    // Initialize BLE controller
    ble_controller_init(configMAX_PRIORITIES - 1);
    extern int hci_driver_init(void);
    // Initialize BLE Host stack
    hci_driver_init();

    tester_send(BTP_SERVICE_ID_CORE, CORE_EV_IUT_READY, BTP_INDEX_NONE,
            NULL, 0);
    #else
    ble_init();
    #endif
#endif

#if defined(CFG_ZIGBEE_ENABLE)

    #ifdef CFG_ZC_REPLACE_ENABLE
    replaced_zc_addr_restore();
    #endif 

    zigbee_init();
#endif
}

void main()
{
    system_init();
    system_thread_init();

#if defined(CONFIG_AUTO_PTS)
    tester_init();
#endif

#ifdef CFG_ETHERNET_ENABLE
    tcpip_init(NULL, NULL);
    lwip_init_netif();
#endif /*CFG_ETHERNET_ENABLE*/
}
