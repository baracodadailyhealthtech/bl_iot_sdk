#include <FreeRTOS.h>
#include <task.h>

#include "bluetooth.h"
#include "ble_cli_cmds.h"
#include "hci_driver.h"
#include "hci_core.h"
#include "log.h"

#include "btble_lib_api.h"
#include "bas.h"
#include "conn.h"
#include "conn_internal.h"
#include "gatt.h"
#include "cli.h"
#include "bl_flash.h"
#include "ble_rc_hog.h"

static void ble_key_notify_process(u8_t hid_page, u8_t *hid_usage, bool press, bool auto_release);
static void ble_key_notify(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
             
const struct cli_command bleFireTvRcCmdSet[] STATIC_CLI_CMD_ATTRIBUTE = {
      {"ble_key_ch_ins", "", ble_key_notify},
      {"ble_key_ch_des", "", ble_key_notify},
      {"ble_key_right", "", ble_key_notify},
      {"ble_key_left", "", ble_key_notify},
      {"ble_key_down", "", ble_key_notify},
      {"ble_key_up", "", ble_key_notify},
      {"ble_key_menu", "", ble_key_notify},
      {"ble_key_pwr", "", ble_key_notify},
      {"ble_key_pick", "", ble_key_notify},
      {"ble_key_mute", "", ble_key_notify},
      {"ble_key_home", "", ble_key_notify},
      {"ble_key_back", "", ble_key_notify},
      {"ble_key_vol_ins", "", ble_key_notify},
      {"ble_key_vol_des", "", ble_key_notify},
};

struct app_env_tag{
    u8_t app_state;
    bool bonded;
    bt_security_t sec_level;
} __packed;

struct app_env_tag env;
static struct bt_gatt_exchange_params exchange_params;
extern struct bt_conn *default_conn;

int ble_start_adv(void);
void ble_stack_start(void);

static void exchange_func(struct bt_conn *conn, u8_t err,
    struct bt_gatt_exchange_params *params)
{
    printf("Exchange %s MTU Size =%d \r\n", err == 0U ? "successful" : "failed",bt_gatt_get_mtu(conn));
}

static void bt_gattc_exchange_mtu(void)
{    
    exchange_params.func = exchange_func;
    bt_gatt_exchange_mtu(default_conn, &exchange_params);
}

static void connected(struct bt_conn *conn, u8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        printf("Failed to connect to %s (%u) \r\n", addr, err);
        return;
    }

    printf("Connected: %s \r\n", addr);

    if (!default_conn) {
        default_conn = conn;
    }
    
    env.bonded = false;
}

static void disconnected(struct bt_conn *conn, u8_t reason)
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

    if (default_conn == conn) {
        default_conn = NULL;
    }
}

static void auth_cancel(struct bt_conn *conn)
{
    
	if (default_conn) {
		conn = default_conn;
	}else {
		conn = NULL;
	}

	if (!conn) {
        printf("Not connected\r\n");
		return;
	}

	bt_conn_auth_cancel(conn);
}

static void auth_pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm pairing for %s\r\n", addr);
         bt_conn_auth_pairing_confirm(conn);
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
    printf("auth_pairing_complete\r\n");

    env.bonded = true;
    bt_gattc_exchange_mtu();
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Pairing failed with %s\r\n", addr);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = NULL,
	.passkey_entry = NULL,
	.passkey_confirm = NULL,
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static void ble_key_notify_process(u8_t hid_page, u8_t *hid_usage, bool press, bool auto_release)
{
     int err = 0;

    if(!default_conn){
        printf("Not connected\r\n");
        return;
    }

    err = bt_hog_notify(default_conn, hid_page, hid_usage, press);
    if(auto_release && err == 0)
    {
        err = bt_hog_notify(default_conn, hid_page, hid_usage, 0);
    }

    if(err)
    {
        printf("Notification sent failed\r\n");
    }
    else
    {    
        printf("Notification sent successfully\r\n");
    }
}

static void ble_key_notify(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8_t *hid_usage = NULL;
    u8_t hid_page = HID_PAGE_KBD;
    bool auto_release = true;
    bool press = true;
    if(0==strcmp("ble_key_ch_ins",argv[0]))
    {
        hid_usage = KEY_CH_INS;
    }
    else if(0==strcmp("ble_key_ch_des",argv[0]))
    {
        hid_usage = KEY_CH_DES;
    }
    else if(0==strcmp("ble_key_right",argv[0]))
    {
        hid_usage = KEY_RIGHT;
    }
    else if(0==strcmp("ble_key_left",argv[0]))
    {
        hid_usage = KEY_LEFT;
    }
    else if(0==strcmp("ble_key_down",argv[0]))
    {
        hid_usage = KEY_DOWN;
    }
    else if(0==strcmp("ble_key_up",argv[0]))
    {
        hid_usage = KEY_UP;
    }
    else if(0==strcmp("ble_key_menu",argv[0]))
    {
        hid_usage = KEY_MENU;
    }
    else if(0==strcmp("ble_key_pwr",argv[0]))
    {
        hid_usage = KEY_PWR;
    }
    else if(0==strcmp("ble_key_pick",argv[0]))
    {
        hid_page = HID_PAGE_CONS;
        hid_usage = KEY_PICK;
    }
    else if(0==strcmp("ble_key_mute",argv[0]))
    {
        hid_page = HID_PAGE_CONS;
        hid_usage = KEY_MUTE;
    }
    else if(0==strcmp("ble_key_home",argv[0]))
    {
        hid_page = HID_PAGE_CONS;
        hid_usage = KEY_HOME;
    }
    else if(0==strcmp("ble_key_back",argv[0]))
    {
        hid_page = HID_PAGE_CONS;
        hid_usage = KEY_BACK;
    }
    else if(0==strcmp("ble_key_vol_ins",argv[0]))
    {
        hid_page = HID_PAGE_CONS;
        hid_usage = KEY_VOL_INS;
    }
    else if(0==strcmp("ble_key_vol_des",argv[0]))
    {
        hid_page = HID_PAGE_CONS;
        hid_usage = KEY_VOL_DES;
    }
    else
    {
        printf("Faild to find hid usage");
        return;
    }
    
    ble_key_notify_process(hid_page, hid_usage, press, auto_release);
}

int ble_cli_rc_register(void)
{
    // static command(s) do NOT need to call aos_cli_register_command(s) to register.
    // However, calling aos_cli_register_command(s) here is OK but is of no effect as cmds_user are included in cmds list.
    // XXX NOTE: Calling this *empty* function is necessary to make cmds_user in this file to be kept in the final link.
    //aos_cli_register_commands(btStackCmdSet, sizeof(btStackCmdSet)/sizeof(btStackCmdSet[0]));
    return 0;
}

int ble_start_adv(void)
{    
    struct bt_le_adv_param adv_param = {
        //options:3, connectable undirected, adv one time
        .options = 3, \
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_3, \
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_3, \
    };

    char *adv_name = "702L_RC";
    u8_t data[1] = {(BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)};
    u8_t data_uuid[2] = {0x12, 0x18};//0x1812
    u8_t data_appearance[2] = {0x80, 0x01};//0x0180
    struct bt_data adv_data[] = {
            BT_DATA(BT_DATA_FLAGS, data, 1),
            BT_DATA(BT_DATA_UUID16_ALL, data_uuid, sizeof(data_uuid)),
            BT_DATA(BT_DATA_GAP_APPEARANCE, data_appearance, sizeof(data_appearance)),
            BT_DATA(BT_DATA_NAME_COMPLETE, adv_name, strlen(adv_name)),
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
        ble_cli_rc_register();

        bas_init();
        hog_init();

        env.bonded = false;

        bt_conn_auth_cb_register(&auth_cb_display);
        bt_conn_cb_register(&conn_callbacks);

        if(ble_start_adv() == 0)
        {
            printf("ble advertising is started\r\n");
        }
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
