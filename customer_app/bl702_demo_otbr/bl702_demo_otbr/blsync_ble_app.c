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
#include <FreeRTOS.h>
#include <task.h>

#include <cli.h>

#include <bluetooth.h>
#include <hci_driver.h>
#include <ble_lib_api.h>
#include <ble_cli_cmds.h>
#include <blsync_ble.h>

#include <virt_net.h>
#include <pkg_protocol.h>

#include "main.h"
#include "blsync_ble_app.h"

static bl_ble_sync_t *gp_index = NULL;
static void(*wifi_scan_callback)(void *);

static void wifiprov_connect_ap_ind(struct wifi_conn *info)
{
	char * ap_cred[3];

	ap_cred[1] = (char *)info->ssid;
	ap_cred[2] = (char *)info->pask;
    cmd_connect(NULL, 0, sizeof(ap_cred) / sizeof(ap_cred[0]), ap_cred);

    printf("Recevied indication to connect to AP: %s, %s\r\n", ap_cred[1], ap_cred[2]);    
}

static void wifiprov_disc_from_ap_ind(void)
{
    cmd_disconnect(NULL, 0, 0, NULL);
    printf("Recevied indication to disconnect to AP\r\n");
}

static void wifiprov_wifi_scan(void(*complete)(void *))
{
    printf("Recevied indication to wifi scan\r\n");
    cmd_scan(NULL, 0, 0, NULL);
    wifi_scan_callback = complete;
}

static void wifiprov_api_state_get(void(*state_get)(void *))
{
    printf("Recevied indication to wifi state get\r\n");
}

static void blesync_complete_cb (void *p_arg)
{
    bl_ble_sync_t *p_index = (bl_ble_sync_t *)p_arg;
    bl_ble_sync_stop(p_index);
    vPortFree(p_index);
}

void blesync_wifi_scan_done(netbus_fs_scan_ind_cmd_msg_t *msg)
{
    blesync_wifi_item_t wifi_item;

    for (int i = 0; i < msg->num; i ++) {
        if (wifi_scan_callback) {
            wifi_item.auth = msg->records[i].auth_mode;
            wifi_item.rssi = msg->records[i].rssi;
            wifi_item.channel = msg->records[i].channel;
            wifi_item.ssid_len = strlen((char*)msg->records[i].ssid);
            memcpy(wifi_item.ssid, msg->records[i].ssid, sizeof(wifi_item.ssid));
            wifi_item.ssid[wifi_item.ssid_len] = 0;
            memcpy(wifi_item.bssid, msg->records[i].bssid, sizeof(wifi_item.bssid));
            wifi_scan_callback(&wifi_item);
        }

        printf("%s\t%d\t%d\t%d\t%d\t%d\t\r\n", msg->records[i].ssid
            , msg->records[i].channel, msg->records[i].rssi, msg->records[i].channel
            , msg->records[i].auth_mode, msg->records[i].cipher);
    }
}

static struct blesync_wifi_func WifiProv_conn_callback = {
	.local_connect_remote_ap = wifiprov_connect_ap_ind,
	.local_disconnect_remote_ap = wifiprov_disc_from_ap_ind,
	.local_wifi_scan = wifiprov_wifi_scan,
	.local_wifi_state_get = wifiprov_api_state_get,
};

static void blsync_init(int err)
{
    static const char ble_init_cmd[] = "ble_init\r\n";
    static const char ble_start_adv_cmd[] = "ble_start_adv 0 0 0x100 0x100\r\n";

    if (gp_index) {
    	bl_ble_sync_start(gp_index, &WifiProv_conn_callback,
						  blesync_complete_cb, (void *)gp_index);

        aos_cli_input_direct((char *)ble_init_cmd, strlen(ble_init_cmd));
        aos_cli_input_direct((char *)ble_start_adv_cmd, strlen(ble_start_adv_cmd));
    }
}

void blsync_ble_start (void)
{
    ble_cli_register();

    gp_index = pvPortMalloc(sizeof(bl_ble_sync_t));
    if (gp_index == NULL) {
        return;
    }

    ble_controller_init(configMAX_PRIORITIES - 1);
    hci_driver_init();

    bt_enable(blsync_init);
}

void blsync_ble_stop (void)
{
	if (gp_index) {
	    bl_ble_sync_stop(gp_index);
	    vPortFree(gp_index);
	    gp_index = NULL;
    }
}