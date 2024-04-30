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
#include <bl_rtc.h>
#include <hal_tcal.h>

#include <cli.h>

#include <lwip/tcpip.h>

#include <netutils/netutils.h>

#include <openthread/dataset_ftd.h>
#if OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE
#include <openthread/border_router.h>
#endif /* OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE */
#if OPENTHREAD_CONFIG_BACKBONE_ROUTER_ENABLE
#include <openthread/backbone_router_ftd.h>
#endif /* OPENTHREAD_CONFIG_BORDER_ROUTING_ENABLE */
#include <openthread_port.h>

#include <main.h>

void vApplicationMallocFailedHook(void)
{
#if defined(CFG_USE_PSRAM)
    printf("Memory Allocate Failed. SRAM left: %d bytes, PSRAM left: %d bytes\r\n", 
            xPortGetFreeHeapSize(), xPortGetFreeHeapSizePsram());
#else
    printf("Memory Allocate Failed. SRAM left: %d bytes\r\n", xPortGetFreeHeapSize());
#endif

    while (1) {
        /*empty here*/
    }
}

#if defined(CFG_USB_CDC_ENABLE)
void vApplicationTickHook( void )
{
    extern void usb_cdc_monitor(void);
    usb_cdc_monitor();
}
#endif

void _cli_init(int fd_console)
{
#ifdef SYS_AOS_CLI_ENABLE
    ot_uartSetFd(fd_console);
#endif
#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_start(int fd_console);
    usb_cdc_start(fd_console);
#endif /* CFG_USB_CDC_ENABLE */

    /*Put CLI which needs to be init here*/
#if defined(CFG_EFLASH_LOADER_ENABLE)
    extern int helper_eflash_loader_cli_init(void);
    helper_eflash_loader_cli_init();
#endif

#if defined(CFG_RFPHY_CLI_ENABLE)
    extern int helper_rfphy_cli_init(void);
    helper_rfphy_cli_init();
#endif

    /*Put CLI which needs to be init here*/
    network_netutils_iperf_cli_register();
    network_netutils_ping_cli_register();
}

static void cmd_ipinfo(char *buf, int len, int argc, char **argv)
{
    struct netif  * netif = otbr_getBackboneNetif();
    ip6_addr_t    * ip6addr;
    
    printf("Backbone Address info:\r\n");
    printf("Hwaddr %02X:%02X:%02X:%02X:%02X:%02X\r\n", netif->hwaddr[0], 
        netif->hwaddr[1], netif->hwaddr[2], netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);

    if (netif->flags & NETIF_FLAG_UP) {
#if LWIP_IPV4
        if(!ip4_addr_isany(netif_ip4_addr(netif))){
            printf("IP: %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
            printf("MASK: %s\r\n", ip4addr_ntoa(netif_ip4_netmask(netif)));
            printf("Gateway: %s\r\n", ip4addr_ntoa(netif_ip4_gw(netif)));
        }
#endif /* LWIP_IPV4 */

        for (uint32_t i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i ++ ) {
            if (!ip6_addr_isany(netif_ip6_addr(netif, i)) && 
                ip6_addr_ispreferred(netif_ip6_addr_state(netif, i))) {

                ip6addr = (ip6_addr_t *)netif_ip6_addr(netif, i);
                if (ip6_addr_isany(ip6addr)) {
                    continue;
                }

                if (ip6_addr_islinklocal(ip6addr)) {
                    printf("LOCAL IP6 addr %s\r\n", ip6addr_ntoa(ip6addr));
                }
                else{
                    printf("GLOBAL IP6 addr %s\r\n", ip6addr_ntoa(ip6addr));
                }
            }
        }
    }
    else{
        printf("netif isn't up\r\n");
    }

    netif = otbr_getThreadNetif();
    printf("Thread Address info:\r\n");
    printf("Hwaddr %02X:%02X:%02X:%02X:%02X:%02X\r\n", netif->hwaddr[0], 
        netif->hwaddr[1], netif->hwaddr[2], netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
    if (netif->flags & NETIF_FLAG_UP) {
#if LWIP_IPV4
        if(!ip4_addr_isany(netif_ip4_addr(netif))){
            printf("IP: %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
            printf("MASK: %s\r\n", ip4addr_ntoa(netif_ip4_netmask(netif)));
            printf("Gateway: %s\r\n", ip4addr_ntoa(netif_ip4_gw(netif)));
        }
#endif /* LWIP_IPV4 */

        for (uint32_t i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i ++ ) {
          if (!ip6_addr_isany(netif_ip6_addr(netif, i)) && 
              ip6_addr_ispreferred(netif_ip6_addr_state(netif, i))) {
                
                ip6addr = (ip6_addr_t *)netif_ip6_addr(netif, i);
                if (ip6_addr_isany(ip6addr)) {
                    continue;
                }

                if (ip6_addr_islinklocal(ip6addr)) {
                    printf("LOCAL IP6 addr %s\r\n", ip6addr_ntoa(ip6addr));
                }
                else{
                    printf("GLOBAL IP6 addr %s\r\n", ip6addr_ntoa(ip6addr));
                }
            }
        }
    }
    else{
        printf("netif isn't up\r\n");
    }
}

static void cmd_ipaddr(char *buf, int len, int argc, char **argv)
{
    bool isAdd = false;
    int index = 0;
    ip6_addr_t ip6addr;

    if (argc < 4) {
        return;
    }

    if (0 == strcmp(argv[1], "add")) {
        isAdd = true;
    }
    else if (0 == strcmp(argv[1], "del")) {
        isAdd = false;
    }
    else {
        return;
    }

    index = atoi(argv[2]);
    if (index >= LWIP_IPV6_NUM_ADDRESSES) {
        return;
    }

    if (!isAdd) {
        if (index) {
            netif_ip6_addr_set_state(otbr_getBackboneNetif(), index, IP6_ADDR_INVALID);
        }
        return;
    }

    if (ip6addr_aton(argv[3], &ip6addr)) {
        netif_ip6_addr_set(otbr_getBackboneNetif(), index, &ip6addr);
        netif_ip6_addr_set_state(otbr_getBackboneNetif(), index, IP6_ADDR_PREFERRED);

        otbr_instance_routing_init();
    }
}

const static struct cli_command cmds_user[] STATIC_CLI_CMD_ATTRIBUTE = {
#ifdef CFG_USE_WIFI_BR
    {"wifi_connect", "connect to Wi-Fi AP", cmd_connect},
    {"wifi_disconnect", "wifi disconnect from Wi-Fi AP", cmd_disconnect},
    {"wifi_scan", "wifi scan", cmd_scan},
    {"get_net_info", "get_net_info", cmd_get_info},
#endif /* CFG_USE_WIFI_BR */
    {"ipinfo", "eth ipaddr", cmd_ipinfo},
    {"ipaddr", "ipaddr operation", cmd_ipaddr},
};

void otrAppProcess(ot_system_event_t sevent) 
{
    /** for application code */
    /** Note,   NO heavy execution, no delay and semaphore pending here.
     *          do NOT stop/suspend this task */

}

#ifdef CFG_THREAD_AUTO_START
void otr_start_default(void) 
{
    otOperationalDataset ds;
    uint8_t default_network_key[] = THREAD_NETWORK_KEY;
    uint8_t default_extend_panid[] = THREAD_EXTPANID;

    if (!otDatasetIsCommissioned(otrGetInstance())) {

        if (OT_ERROR_NONE != otDatasetCreateNewNetwork(otrGetInstance(), &ds)) {
            printf("Failed to create dataset for Thread Network\r\n");
        }

        memcpy(&ds.mNetworkKey, default_network_key, sizeof(default_network_key));
        strncpy(ds.mNetworkName.m8, "OTBR-BL702", sizeof(ds.mNetworkName.m8));
        memcpy(&ds.mExtendedPanId, default_extend_panid, sizeof(default_extend_panid));
        ds.mChannel = THREAD_CHANNEL;
        ds.mPanId = THREAD_PANID;
        
        if (OT_ERROR_NONE != otDatasetSetActive(otrGetInstance(), &ds)) {
            printf("Failed to set active dataset\r\n");
        }
    }

    otIp6SetEnabled(otrGetInstance(), true);
    otThreadSetEnabled(otrGetInstance(), true);
}
#endif

void otrInitUser(otInstance * instance)
{
    otAppCliInit((otInstance * )instance);

#ifdef CFG_THREAD_AUTO_START
    otr_start_default();
#endif
    
    otbr_netif_init();
}

int main(int argc, char *argv[])
{
    otRadio_opt_t opt;

    bl_rtc_init();
    hal_tcal_init();

#if defined(CFG_USE_WIFI_BR) && defined (WIFI_LWIP_RESET_PIN) 
    wifi_lwip_hw_reset();
#endif /* CFG_USE_WIFI_BR */

    opt.byte = 0;

#if OPENTHREAD_FTD
    opt.bf.isFtd = true;
#endif
#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    opt.bf.isLinkMetricEnable = true;
#endif
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    opt.bf.isCSLReceiverEnable = true;
#endif
#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    opt.bf.isTimeSyncEnable = true;
#endif

    tcpip_init(NULL, NULL);

#ifdef CFG_USE_WIFI_BR
    wifi_lwip_init();
#else
    eth_lwip_init();
#endif /*CFG_ETHERNET_ENABLE*/

    otrStart(opt);

    return 0;
}
