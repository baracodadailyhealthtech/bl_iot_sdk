#include <stdint.h>
#include "bl_irq.h"
#include "btble_lib_api.h"
#include "bluetooth.h"
#include "conn.h"
#include "conn_internal.h"
#include "hci_driver.h"
#include "hci_core.h"
#include "bt_log.h"
#include "bl_port.h"
#if defined(CONFIG_BT_STACK_CLI) 
#include "ble_cli_cmds.h"
#endif
#include "ble_app.h"
#include "pds_app.h"
#if defined(CONFIG_BLE_TP_SERVER)
#include "ble_tp_svc.h"
#endif

#define BLE_APP_DEV_NAME "BL702L_BLE"
#define BLE_APP_ADV_INT_MIN 0x00A0 //100ms
#define BLE_APP_ADV_INT_MAX 0x00A0 //100ms

struct bt_conn *bleapp_default_conn;

static void bleapp_connected(struct bt_conn *conn, u8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        printf("Failed to connect to %s (%u) \r\n", addr, err);
        return;
    }

    printf("Connected: %s, int 0x%04x lat %d to %d\r\n", addr, conn->le.interval, conn->le.latency, conn->le.timeout);

    if (!bleapp_default_conn) {
        bleapp_default_conn = conn;
    }
}

static void bleapp_disconnected(struct bt_conn *conn, u8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Disconnected: %s (reason %u) \r\n", addr, reason);

    if(conn->role == BT_CONN_ROLE_SLAVE) {
        if(set_adv_enable(true)) {
            printf("Fail to restart adv. \r\n");
        } else {
            printf("Restart adv successfully. \r\n");
        }
    }

    if (bleapp_default_conn == conn) {
        bleapp_default_conn = NULL;
    }
}

static void bleapp_auth_cancel(struct bt_conn *conn)
{
    
	if (bleapp_default_conn) {
		conn = bleapp_default_conn;
	}else {
		conn = NULL;
	}

	if (!conn) {
        printf("Not connected\r\n");
		return;
	}

	bt_conn_auth_cancel(conn);
}

static void bleapp_auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));    

    printf("passkey_str is: %06u\r\n", passkey);
}

static void bleapp_auth_passkey_entry(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Enter passkey for %s\r\n", addr);
}

static void  bleapp_auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm passkey for %s: %06u\r\n", addr, passkey);

    bt_conn_auth_passkey_confirm(bleapp_default_conn);
}

static void bleapp_auth_pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm pairing for %s\r\n", addr);
         bt_conn_auth_pairing_confirm(conn);

    bt_conn_auth_pairing_confirm(bleapp_default_conn);
}

static void bleapp_auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
    printf("auth_pairing_complete\r\n");
}

static void bleapp_conn_param_updated(struct bt_conn *conn, u16_t interval,
			     u16_t latency, u16_t timeout)
{
    if(conn == bleapp_default_conn)
    {
        printf("%s: int 0x%04x lat %d to %d \r\n", __func__, interval, latency, timeout);
    }
}

static void bleapp_auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Pairing failed with %s\r\n", addr);
}

static void bleapp_identity_resolved(struct bt_conn *conn, const bt_addr_le_t *rpa,
			      const bt_addr_le_t *identity)
{
    char addr_identity[BT_ADDR_LE_STR_LEN];
    char addr_rpa[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(identity, addr_identity, sizeof(addr_identity));
    bt_addr_le_to_str(rpa, addr_rpa, sizeof(addr_rpa));

    printf("Identity resolved %s -> %s \r\n", addr_rpa, addr_identity);

}

static void bleapp_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Security changed: %s level %u \r\n", addr, level);
}

static struct bt_conn_cb bleapp_conn_callbacks = {
    .connected = bleapp_connected,
    .disconnected = bleapp_disconnected,
    .le_param_updated = bleapp_conn_param_updated,
    .identity_resolved = bleapp_identity_resolved,
    .security_changed = bleapp_security_changed,
};

static struct bt_conn_auth_cb bleapp_auth_cb_display = {
    .passkey_display = bleapp_auth_passkey_display,
    .passkey_entry = bleapp_auth_passkey_entry,
    .passkey_confirm = bleapp_auth_passkey_confirm,
    .cancel = bleapp_auth_cancel,
    .pairing_confirm = bleapp_auth_pairing_confirm,
    .pairing_failed = bleapp_auth_pairing_failed,
    .pairing_complete = bleapp_auth_pairing_complete,
};

int bleapp_start_adv(void)
{    
    struct bt_le_adv_param adv_param = {
        //options:3, connectable undirected, adv one time
        .options = 3, \
        .interval_min = BLE_APP_ADV_INT_MIN, \
        .interval_max = BLE_APP_ADV_INT_MAX, \
    };

    char *adv_name = BLE_APP_DEV_NAME;
    u8_t data[1] = {(BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)};
   // u8_t data_uuid[4] = {0x12, 0x18, 0x0F, 0x18};//0x1812,0x180f
    //u8_t data_appearance[2] = {0xc1, 0x03};//0x0180 {0x80, 0x01};
    //u8_t data_manuf_data[15] = {0xe0, 0x00, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    struct bt_data adv_data[] = {
            BT_DATA(BT_DATA_FLAGS, data, 1),
            //BT_DATA(BT_DATA_GAP_APPEARANCE, data_appearance, sizeof(data_appearance)),
            //BT_DATA(BT_DATA_UUID16_SOME, data_uuid, sizeof(data_uuid)),
            BT_DATA(BT_DATA_NAME_COMPLETE, adv_name, strlen(adv_name)),
            //BT_DATA(BT_DATA_MANUFACTURER_DATA, data_manuf_data, sizeof(data_manuf_data)),
        };

    return bt_le_adv_start(&adv_param, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}


void bt_enable_cb(int err)
{
    if (!err) {
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);

        bt_conn_auth_cb_register(&bleapp_auth_cb_display);
        bt_conn_cb_register(&bleapp_conn_callbacks);
        #if defined(CONFIG_BLE_TP_SERVER)
        ble_tp_init();
        #endif
        #if defined(CONFIG_BT_STACK_CLI) 
        ble_cli_register();
        #endif
        bt_set_name(BLE_APP_DEV_NAME);
        if(bleapp_start_adv() == 0)
        {
            printf("ble advertising is started\r\n");
        }
    }
}

void ble_stack_start(void)
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
