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

#include <eth_bd.h>

#include <aos/yloop.h>

#include <lwip/dhcp.h>
#include <lwip/dhcp6.h>
#include <lwip/netifapi.h>
#include <netif/ethernet.h>

#include "main.h"

extern err_t eth_init(struct netif *netif);

static struct dhcp6 dhcp6_val;

struct netif * otbr_getBackboneNetif(void) 
{
    return &eth_mac;
}

static int app_eth_callback(eth_link_state val)
{
    switch(val){
    case ETH_INIT_STEP_LINKUP:
        printf("Ethernet link up\r\n");
        break;
    case ETH_INIT_STEP_READY:
        netifapi_netif_set_default(&eth_mac);
        netifapi_netif_set_up(&eth_mac);

        //netifapi_netif_set_up((struct netif *)&obj->netif);
        netif_create_ip6_linklocal_address(&eth_mac, 1);
        eth_mac.ip6_autoconfig_enabled = 1;
        dhcp6_set_struct(&eth_mac, &dhcp6_val);
        dhcp6_enable_stateless(&eth_mac);

        printf("start dhcp...\r\n");

#if LWIP_IPV4
        /* start dhcp */
        netifapi_dhcp_start(&eth_mac);
#endif
        break;
    case ETH_INIT_STEP_LINKDOWN:
        printf("Ethernet link down\r\n");
        break;
    }

    return 0;
}

static void netif_status_callback(struct netif *netif)
{
    bool isIPv6AddressAssigend = false;

    if (netif->flags & NETIF_FLAG_UP) {
#if LWIP_IPV4
        if(!ip4_addr_isany(netif_ip4_addr(netif))) {
            printf("IP: %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
            printf("MASK: %s\r\n", ip4addr_ntoa(netif_ip4_netmask(netif)));
            printf("Gateway: %s\r\n", ip4addr_ntoa(netif_ip4_gw(netif)));
        }
#endif

        for (uint32_t i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i ++ ) {
            if (!ip6_addr_isany(netif_ip6_addr(netif, i))
                && ip6_addr_ispreferred(netif_ip6_addr_state(netif, i))) {

                const ip6_addr_t* ip6addr = netif_ip6_addr(netif, i);
                if (ip6_addr_isany(ip6addr)) {
                    continue;
                }

                if(ip6_addr_islinklocal(ip6addr)){
                    printf("LOCAL IP6 addr %s\r\n", ip6addr_ntoa(ip6addr));
                }
                else{
                    printf("GLOBAL IP6 addr %s\r\n", ip6addr_ntoa(ip6addr));
                    isIPv6AddressAssigend = true;
                } 
            }
        }

        if (isIPv6AddressAssigend) {
            main_task_resume();
        }
    }
    else {
        printf("interface is down status.\n");
    }
}

void eth_lwip_init(void)
{
#if LWIP_IPV4
    netif_add(&eth_mac, NULL, NULL, NULL, NULL, eth_init, ethernet_input);
#else
    netif_add(&eth_mac, NULL, eth_init, ethernet_input);
#endif

    ethernet_init(app_eth_callback);

    /* Set callback to be called when interface is brought up/down or address is changed while up */
    netif_set_status_callback(&eth_mac, netif_status_callback);
}
