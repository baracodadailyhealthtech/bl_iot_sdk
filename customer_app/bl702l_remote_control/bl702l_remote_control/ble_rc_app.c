#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "bl_kys.h"
#include "bl_irq.h"
#include "bl_adc.h"
#include "bl_ir.h"
#include "btble_lib_api.h"
#include "bluetooth.h"
#include "hci_driver.h"
#include "hci_core.h"
#include "hci_host.h"
#include "log.h"
#include "btble_lib_api.h"
#include "bas.h"
#include "conn.h"
#include "conn_internal.h"
#include "gatt.h"
#include "bl_port.h"
#include "cli.h"
#include "rom_hal_ext.h"
#include "rom_btble_ext.h"
#include "ble_rc_hog.h"
#include "ble_rc_voice.h"
#include "ble_rc_app.h"
#include "ble_atv_voice.h"

volatile bool voice_start = false;
volatile bool cont_start = false;//send release key value after key is released.
volatile bool cont_release = false;
volatile bool wait_for_unpair = false;
volatile u8_t rc_battery_level = 80;
struct k_fifo ble_rc_key_scan_queue;
static struct bt_gatt_exchange_params exchange_params;
struct bt_conn *rc_default_conn;
struct hids_remote_key *m_key_usage;
static TaskHandle_t ble_rc_Key_Scan_task_hdl;
k_timer_t ble_rc_adv_timer;//restart adv
k_timer_t ble_rc_adc_sample_timer;//adv sample

static int ble_rc_start_adv(void);
static void ble_rc_key_notify_process(u8_t hid_page, u8_t *hid_usage, bool press, bool auto_release);
static void ble_rc_key_notify(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void ble_rc_create_adc_sample_timer(void);
    
const struct cli_command bleFireTvRcCmdSet[] STATIC_CLI_CMD_ATTRIBUTE = {
      {"ble_key_ch_ins", "", ble_rc_key_notify},
      {"ble_key_ch_des", "", ble_rc_key_notify},
      {"ble_key_right", "", ble_rc_key_notify},
      {"ble_key_left", "", ble_rc_key_notify},
      {"ble_key_down", "", ble_rc_key_notify},
      {"ble_key_up", "", ble_rc_key_notify},
      {"ble_key_menu", "", ble_rc_key_notify},
      {"ble_key_pwr", "", ble_rc_key_notify},
      {"ble_key_pick", "", ble_rc_key_notify},
      {"ble_key_mute", "", ble_rc_key_notify},
      {"ble_key_home", "", ble_rc_key_notify},
      {"ble_key_back", "", ble_rc_key_notify},
      {"ble_key_vol_ins", "", ble_rc_key_notify},
      {"ble_key_vol_des", "", ble_rc_key_notify},
      {"ble_key_voice_start", "", ble_rc_key_notify},
      {"ble_key_voice_stop", "", ble_rc_key_notify},
};

int ble_rc_connection_update(u16_t interval_min, u16_t interval_max, u16_t latency, u16_t timeout)
{
    int err = 0;
    struct bt_le_conn_param param;
    param.interval_min = interval_min;
    param.interval_max = interval_max;
    param.latency = latency;
    param.timeout = timeout;
    
    err = bt_conn_le_param_update(rc_default_conn, &param);
    return err;
}

static void ble_rc_data_len_extend(struct bt_conn *conn)
{
    u16_t tx_octets = 0x00fb;
    u16_t tx_time = 0x0848;
    int ret = -1;
    
    if(conn == NULL)
        return;
    //set data length after connected.
    ret = bt_le_set_data_len(conn, tx_octets, tx_time);
    if(!ret)
    {
        printf("ble rc set data length success.");
    }
    else
    {
        printf("ble rc set data length failure, err: %d\n", ret);
    }
}

static void ble_rc_exchange_func(struct bt_conn *conn, u8_t err,
    struct bt_gatt_exchange_params *params)
{
    printf("Exchange %s MTU Size =%d \r\n", err == 0U ? "successful" : "failed",bt_gatt_get_mtu(conn));
    if(!err)
        ble_rc_data_len_extend(rc_default_conn);
}

static void ble_rc_gatt_exchange_mtu(void)
{    
    exchange_params.func = ble_rc_exchange_func;
    bt_gatt_exchange_mtu(rc_default_conn, &exchange_params);
}

static void ble_rc_connected(struct bt_conn *conn, u8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        printf("Failed to connect to %s (%u) \r\n", addr, err);
        return;
    }

    printf("Connected: %s,int 0x%04x lat %d to %d \r\n", addr,conn->le.interval,conn->le.latency,conn->le.timeout);

    if (!rc_default_conn) {
        rc_default_conn = conn;
    }

#if 0
    err = bt_conn_set_security(rc_default_conn, BT_SECURITY_L2);
    if(err){
        printf("Failed to start security, (err %d) \r\n", err);
    }else{
        printf("Start security successfully\r\n");
    }
#endif
#if 0
    struct bt_hci_rp_le_read_chan_map rsp;
    int err_read = bt_le_read_chan_map(conn, &rsp);
    if(!err_read)
    {
        printf("channel map:");
        for(int i = 0; i < 5; i++)
        {
            printf("0x%02x ", rsp.ch_map[i]);
        }
        printf("\r\n");
    }
#endif 
    ble_rc_gatt_exchange_mtu();
    ble_rc_create_adc_sample_timer();
}

static void ble_rc_disconnected(struct bt_conn *conn, u8_t reason)
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

    if (rc_default_conn == conn) {
        rc_default_conn = NULL;
    }
}

static void ble_rc_auth_cancel(struct bt_conn *conn)
{  
    if (rc_default_conn) {
        conn = rc_default_conn;
    }else {
        conn = NULL;
    }
    
    if (!conn) {
        printf("Not connected\r\n");
        return;
    }
    
    bt_conn_auth_cancel(conn);
}

static void ble_rc_auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));    
    printf("passkey_str is: %06u\r\n", passkey);
}

static void ble_rc_auth_passkey_entry(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Enter passkey for %s\r\n", addr);
}

static void  ble_rc_auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Confirm passkey for %s: %06u\r\n", addr, passkey);
    bt_conn_auth_passkey_confirm(rc_default_conn);
}

static void ble_rc_auth_pairing_confirm(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Confirm pairing for %s\r\n", addr);
    bt_conn_auth_pairing_confirm(conn);
}

static void ble_rc_auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
    printf("auth_pairing_complete\r\n");
}

static void ble_rc_conn_param_updated(struct bt_conn *conn, u16_t interval,
			     u16_t latency, u16_t timeout)
{
    if(conn == rc_default_conn)
    {
        printf("%s: int 0x%04x lat %d to %d \r\n", __func__, interval, latency, timeout);

        #if defined (CONFIG_ATVV_SERVER_ENABLE)
        if(ble_atvv_if_voice_start_pending())
        { 
            ble_atvv_voice_start_cont();
        }
        #endif
    }
}

static void ble_rc_auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Pairing failed with %s\r\n", addr);
}

static void ble_rc_identity_resolved(struct bt_conn *conn, const bt_addr_le_t *rpa,
			      const bt_addr_le_t *identity)
{
    char addr_identity[BT_ADDR_LE_STR_LEN];
    char addr_rpa[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(identity, addr_identity, sizeof(addr_identity));
    bt_addr_le_to_str(rpa, addr_rpa, sizeof(addr_rpa));

    printf("Identity resolved %s -> %s \r\n", addr_rpa, addr_identity);
}

static void ble_rc_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printf("Security changed: %s level %u \r\n", addr, level);
}

static struct bt_conn_cb ble_rc_conn_callbacks = {
    .connected = ble_rc_connected,
    .disconnected = ble_rc_disconnected,
    .le_param_updated = ble_rc_conn_param_updated,
    .identity_resolved = ble_rc_identity_resolved,
    .security_changed = ble_rc_security_changed,
};

static struct bt_conn_auth_cb ble_rc_auth_cb_display = {
	.passkey_display = NULL,//ble_rc_auth_passkey_display,
	.passkey_entry = NULL,//ble_rc_auth_passkey_entry,
	.passkey_confirm = NULL,//ble_rc_auth_passkey_confirm,
	.cancel = ble_rc_auth_cancel,
	.pairing_confirm = ble_rc_auth_pairing_confirm,
	.pairing_failed = ble_rc_auth_pairing_failed,
	.pairing_complete = ble_rc_auth_pairing_complete,
};

static void ble_rc_key_notify_process(u8_t hid_page, u8_t *hid_usage, bool press, bool auto_release)
{
    int err = 0;

    if(!rc_default_conn){
        printf("Not connected\r\n");
        return;
    }

    err = bt_hog_notify(rc_default_conn, hid_page, hid_usage, press);

    if(auto_release && err == 0)
    {
        err = bt_hog_notify(rc_default_conn, hid_page, hid_usage, 0);
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

static void ble_rc_key_notify(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8_t *hid_usage = NULL;
    u8_t hid_page = HID_PAGE_KBD;
    bool auto_release = true;
    bool press = true;
    u8_t evt_type = 0;
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
    else if(0==strcmp("ble_key_voice_start",argv[0]))
    {
        voice_start = true;
        evt_type = RC_KYS_VOICE_START;
        printf("put RC_KYS_VOICE_START\r\n");
        k_fifo_put(&ble_rc_key_scan_queue, &evt_type);
        return;
    }
    else if(0==strcmp("ble_key_voice_stop",argv[0]))
    {
       if(voice_start)
        {
            voice_start = false;
            evt_type = RC_KYS_VOICE_STOP;
            k_fifo_put(&ble_rc_key_scan_queue, &evt_type); 
        }
       return;
    }
    else
    {
        printf("Faild to find hid usage");
        return;
    }
    
    ble_rc_key_notify_process(hid_page, hid_usage, press, auto_release);
}

static void ble_rc_adv_timer_cb(void *timer)
{
    if(!wait_for_unpair)
    {
        if(rc_default_conn)
            bt_conn_disconnect(rc_default_conn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
        else
        {
            printf("start adv\r\n");
            set_adv_enable(true);
        }

        wait_for_unpair = true;
        k_timer_start(&ble_rc_adv_timer, pdMS_TO_TICKS((BLE_RC_UNPAIR_TIMER_IN_SEC - BLE_RC_ADV_TIMER_IN_SEC) * 1000));
    }
    else
    {
        k_timer_delete(&ble_rc_adv_timer);
        ble_rc_adv_timer.timer.hdl = NULL;
        wait_for_unpair = false;
        printf("to unpair\r\n");
        bt_unpair(0, NULL);
    }
}

void ble_rc_create_adv_timer(void)
{
    printf("%s\r\n", __func__);
    k_timer_init(&ble_rc_adv_timer, ble_rc_adv_timer_cb, NULL);
    k_timer_start(&ble_rc_adv_timer, pdMS_TO_TICKS(BLE_RC_ADV_TIMER_IN_SEC * 1000));
}

static void ble_rc_key_scan_task(void *pvParameters)
{
   bl_kys_trigger_interrupt();

    while(1)
    {
        u8_t * data = k_fifo_get(&ble_rc_key_scan_queue, K_FOREVER);
        if(data)
        {
            u8_t evt_type = *data;
            printf("evt_type=%d\r\n",evt_type);
            k_free(data);
            
            if(!rc_default_conn && evt_type != RC_KYS_ADV && evt_type != RC_KYS_IR_TX && evt_type!= RC_KYS_VOICE_START && evt_type!= RC_KYS_VOICE_STOP)
            {
                printf("rc is not connected\r\n");
                continue;
            }
            switch(evt_type)
            {
                case RC_KYS_NOTIFY:
                {
                    if(m_key_usage)
                    {
                        if(cont_start)
                        {  
                            if(!cont_release)
                                ble_rc_key_notify_process(m_key_usage->hid_page, m_key_usage->hid_usage, true, false);
                            else
                            {
                                cont_release = false;
                                cont_start = false;
                                ble_rc_key_notify_process(m_key_usage->hid_page, m_key_usage->hid_usage, false, false);
                                m_key_usage = NULL;
                            }
                        }
                        else
                        {
                            ble_rc_key_notify_process(m_key_usage->hid_page, m_key_usage->hid_usage, true, true);
                            m_key_usage = NULL;
                        }
                    }
                }
                break;
                case RC_KYS_ADV:
                {
                    ble_rc_create_adv_timer();
                }
                break;
                case RC_KYS_DELETE_ADV_TIMER:
                {
                    if(ble_rc_adv_timer.timer.hdl)
                    {
                         printf("delete adv timer\r\n");
                         k_timer_delete(&ble_rc_adv_timer);
                         ble_rc_adv_timer.timer.hdl = NULL;
                    }
                }
                break;
                case RC_KYS_VOICE_START:
                {
                    printf("RC_KYS_VOICE_START\r\n");
                    #if defined (CONFIG_ATVV_SERVER_ENABLE)
                    if(rc_default_conn)
                        ble_atvv_voice_start();
                    else
                        ble_rc_voice_start();    
                    #else
                    ble_rc_voice_start();
                    #endif
                }
                break;
                case RC_KYS_VOICE_STOP:
                {
                    printf("RC_KYS_VOICE_STOP\r\n");
                    #if defined (CONFIG_ATVV_SERVER_ENABLE)
                    if(rc_default_conn)
                        ble_atvv_voice_stop();
                    #endif
                    #if (VOICE_TASK_DESTROY_ONCE_VOICE_STOP)
                    if(rc_default_conn)
                        ble_rc_voice_task_destroy();
                    #endif
                }
                break;
                case RC_KYS_REPORT_BATTERY_LEVEL:
                {
                    int ret = bt_gatt_bas_set_battery_level(rc_default_conn, rc_battery_level);
                    if(ret)
                        printf("Fails to report battery(err=%d)\r\n", ret);
                    else
                        printf("Report battery level\r\n");
                }
                break;
                case RC_KYS_IR_TX:
                {
                    printf("ble_rc_ir_tx_demo\r\n");
                    extern void ble_rc_ir_tx_demo(void);
                    ble_rc_ir_tx_demo();
                }
                break;

                default:
                break;
            }    
        }
    }
}

static struct hids_remote_key *key_usage[4 /* row */][4 /* col */] = {
    {&remote_kbd_map_tab[4], &remote_kbd_map_tab[0],  &remote_kbd_map_tab[1], &remote_kbd_map_tab[2]  },
    {&remote_kbd_map_tab[3], NULL ,                   &remote_kbd_map_tab[5], &remote_kbd_map_tab[6]  },
    {&remote_kbd_map_tab[7], &remote_kbd_map_tab[8],  &remote_kbd_map_tab[9], &remote_kbd_map_tab[10] },
    {&remote_kbd_map_tab[11], &remote_kbd_map_tab[12],&remote_kbd_map_tab[13],NULL                    },
};

ATTR_PDS_SECTION bool bl_kys_check_existed(const kys_result_t *result, u8_t row_idx, u8_t col_idx)
{
    for(int i = 0; i < result->key_num; i++)
    {
        if(result->row_idx[i] == row_idx && result->col_idx[i] == col_idx)
            return true;
    }

    return false;
}

ATTR_PDS_SECTION void bl_kys_interrupt_callback(const kys_result_t *result)
{  
    static bool pressed = false;
    static bool adv_key_pressed = false;
    static bool adv_key_released = false;
    u8_t home_key_row_idx = 3, home_key_col_idx = 1;
    u8_t back_key_row_idx = 1, back_key_col_idx = 3;
    u8_t voice_key_row_idx = 1, voice_key_col_idx = 1;
    u8_t ir_row_idx = 3, ir_col_idx = 3;
    bool process = false;
    u8_t *evt_type_ptr = NULL;

    bl_pds_set_white_keys(result->key_num, result->row_idx, result->col_idx);
    if(result->key_num && !pressed)
    {
        pressed = true;
        process = true;
    }

    if(result->key_num == 0)
    { 
         if(adv_key_pressed)
        {
            process = true;
            adv_key_released = true;
            adv_key_pressed = false;
            cont_start = false;
        }
        else if(pressed && (voice_start || cont_start))
        {
            process = true;
            if(cont_start)
            {
                cont_release = true;
            }
        }

        pressed = false;
    }
    else if(result->key_num == 2 && adv_key_pressed == false)
    {
        if(bl_kys_check_existed(result, home_key_row_idx,home_key_col_idx) &&
           bl_kys_check_existed(result, back_key_row_idx,back_key_col_idx))
        {
           adv_key_pressed = true;
           process = true;
        }
    }
    
    if(process)
    { 
        flash_restore(); 
        if(voice_start)
        {   
            #if defined (CONFIG_ATVV_SERVER_ENABLE)
            if(!rc_default_conn || ble_atvv_get_assist_mode() == ATVV_ASSIS_MODEL_HTT)
            #endif
            {
                ble_rc_pds_enable(1);
                ble_rc_voice_stop();
                voice_start = false;
                evt_type_ptr = k_malloc(1);
                *evt_type_ptr = RC_KYS_VOICE_STOP;
                k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
            }
        }
        else if(cont_release)
        {
            evt_type_ptr = k_malloc(1);
            *evt_type_ptr = RC_KYS_NOTIFY;
            k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
        }
        else if(adv_key_pressed)
        {
            evt_type_ptr = k_malloc(1);
            *evt_type_ptr = RC_KYS_ADV;
            k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
        }
        else if(adv_key_released && ble_rc_adv_timer.timer.hdl)
        {
            adv_key_released = false;
            evt_type_ptr = k_malloc(1);
            *evt_type_ptr = RC_KYS_DELETE_ADV_TIMER;
            k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
        }
        else if(result->key_num == 1)
        {
            printf("key(%d, %d)\r\n", result->row_idx[0], result->col_idx[0]);
            if(result->row_idx[0] == voice_key_row_idx && result->col_idx[0] == voice_key_col_idx)
            {
                voice_start = true;
                ble_rc_pds_enable(0);
                evt_type_ptr = k_malloc(1);
                *evt_type_ptr = RC_KYS_VOICE_START;
                k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
            }
            else if(result->row_idx[0] == ir_row_idx && result->col_idx[0] == ir_col_idx)
            {
                evt_type_ptr = k_malloc(1);
                *evt_type_ptr = RC_KYS_IR_TX;
                k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
            }
            else
            {
                evt_type_ptr = k_malloc(1);
                *evt_type_ptr = RC_KYS_NOTIFY;
                m_key_usage = key_usage[result->row_idx[0]][result->col_idx[0]];
                cont_start = true;
                cont_release = false;
                   
                k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
            }
        } 
    }
   
    bl_kys_trigger_interrupt();
}

void ble_rc_create_key_scan_task(void)
{
    printf("%s\r\n",__func__);
    k_fifo_init(&ble_rc_key_scan_queue, 10);
    xTaskCreate(ble_rc_key_scan_task, "ble_key_scan_task", (3*1024)/sizeof(StackType_t), (void *)NULL, 20, // max is 31 ( configMAX_PRIORITIES - 1 )
                &ble_rc_Key_Scan_task_hdl);
}

int ble_rc_cli_register(void)
{
    // static command(s) do NOT need to call aos_cli_register_command(s) to register.
    // However, calling aos_cli_register_command(s) here is OK but is of no effect as cmds_user are included in cmds list.
    // XXX NOTE: Calling this *empty* function is necessary to make cmds_user in this file to be kept in the final link.
    //aos_cli_register_commands(btStackCmdSet, sizeof(btStackCmdSet)/sizeof(btStackCmdSet[0]));
    return 0;
}

int ble_rc_start_adv(void)
{    
    struct bt_le_adv_param adv_param = {
        //options:3, connectable undirected, adv one time
        .options = 3, \
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_3, \
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_3, \
    };

    char *adv_name = BLE_RC_NAME;
    u8_t data[1] = {(BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)};
    u8_t data_uuid[4] = {0x12, 0x18, 0x0F, 0x18};//0x1812,0x180f
    u8_t data_appearance[2] = {0xc1, 0x03};//0x0180 {0x80, 0x01};
    //u8_t data_manuf_data[15] = {0xe0, 0x00, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    struct bt_data adv_data[] = {
            BT_DATA(BT_DATA_FLAGS, data, 1),
            BT_DATA(BT_DATA_GAP_APPEARANCE, data_appearance, sizeof(data_appearance)),
            BT_DATA(BT_DATA_UUID16_SOME, data_uuid, sizeof(data_uuid)),
            BT_DATA(BT_DATA_NAME_COMPLETE, adv_name, strlen(adv_name)),
            //BT_DATA(BT_DATA_MANUFACTURER_DATA, data_manuf_data, sizeof(data_manuf_data)),
        };

    return bt_le_adv_start(&adv_param, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
}

ATTR_PDS_SECTION void ble_rc_kys_init(void)
{
    static bool initiated = false;
    //gpio 0 and 1 are JTAG pins,gpio 30 and 32 are used by xtal32k if xtal32k exists on the board.
    uint8_t row_pins[] = {/*31, 30*/ 10, 25, 24, 23};
    uint8_t col_pins[] = {9, /*1,0,*/ 28, 27, 26};
    bl_kys_init(sizeof(row_pins), sizeof(col_pins), row_pins, col_pins);
    #if defined(CFG_BLE_PDS)
    if(!initiated)
    {
        bl_pds_key_wakeup_cfg(sizeof(row_pins), sizeof(col_pins), row_pins, col_pins);
        initiated = true;
    }
    #endif
}

ATTR_PDS_SECTION int ble_rc_before_sleep_callback(void)
{
    taskENTER_CRITICAL();
    //disable keyscan
    bl_kys_abort();
    //clear keyscan irq pending bit.
    rom_bl_irq_pending_clear(KYS_IRQn);
    taskEXIT_CRITICAL();

#if 0
    uint32_t pullup_bitmap = 1<<11;  // enable GPIO11 pull-up when entering pds, and still take effect after wakeup
    uint32_t pulldown_bitmap = 1<<12;  // enable GPIO12 pull-down when entering pds, and still take effect after wakeup
    bl_pds_gpio_pull_set(pullup_bitmap, pulldown_bitmap);
#endif

    return 0;
}

ATTR_PDS_SECTION void ble_rc_sleep_aborted_callback(void)
{
    //enable keyscan
    bl_kys_trigger_interrupt();
}

ATTR_PDS_SECTION void ble_rc_after_sleep_callback(void)
{
    uint8_t key_row_idx = 0xff, key_col_idx = 0xff;
    uint8_t key_evt_type = 0;

    // disable GPIO pull-up/pull-down, better after GPIO reinitialization
    bl_pds_gpio_pull_disable();

    uart_init(14, 15, 2000000);
    if(bl_pds_get_wakeup_source() == PDS_WAKEUP_BY_KEY)
    {
        key_evt_type = bl_pds_get_wakeup_key_index(&key_row_idx, &key_col_idx);
        printf("wakeup source: key -> (%d, %d),evt=%d\r\n", key_row_idx, key_col_idx, key_evt_type);
    }
    //enable keyscan
    ble_rc_kys_init();
    bl_kys_trigger_interrupt();
}

static void ble_rc_adc_timer_cb()
{
    printf("%s\r\n", __func__);
    float val = 0;
    bl_adc_vbat_init();
    val = bl_adc_vbat_get_val();
    printf("%s, val = %f\r\n", __func__, val);

    /*
    to do:transfer to  rc_battery_level
    */
    
    u8_t *evt_type_ptr = k_malloc(1);
    *evt_type_ptr = RC_KYS_REPORT_BATTERY_LEVEL;
    k_fifo_put_from_isr(&ble_rc_key_scan_queue, evt_type_ptr);
    k_timer_start(&ble_rc_adc_sample_timer, pdMS_TO_TICKS(BLE_RC_ADC_TIMER_IN_SEC * 1000));
}
    
static void ble_rc_create_adc_sample_timer(void)
{ 
    printf("%s\r\n", __func__);
    k_timer_init(&ble_rc_adc_sample_timer, ble_rc_adc_timer_cb, NULL);
    k_timer_start(&ble_rc_adc_sample_timer, pdMS_TO_TICKS(BLE_RC_ADC_TIMER_IN_SEC * 1000));
}

void ble_rc_pds_enable(uint8_t enable)
{
    btble_pds_enable(enable);
}

void ble_rc_foreach_bond_info_cb(const struct bt_bond_info *info, void *user_data)
{
    char addr[BT_ADDR_LE_STR_LEN];
    u8_t bonded_device_cnt = 0;
    if(user_data)
        (*(u8_t *)user_data)++;
    bt_addr_le_to_str(&info->addr, addr, sizeof(addr));
    printf("bonded device: %s\r\n", addr);
}

void bt_enable_cb(int err)
{
    if (!err) {
        u8_t bonded_device_cnt = 0;
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);
        ble_rc_cli_register();
        bt_set_name(BLE_RC_NAME);
        bt_gap_set_local_device_appearance(BLE_RC_DEVICE_APPEARANCE);
        bas_init();
        hog_init();
        ble_rc_create_key_scan_task();

        bt_conn_auth_cb_register(&ble_rc_auth_cb_display);
        bt_conn_cb_register(&ble_rc_conn_callbacks);

        //If there is no bonded device, ble_rc_foreach_bond_info_cb will not be called.
        //If there is N bonded devices, ble_rc_foreach_bond_info_cb will be called N times.
        bt_foreach_bond(0, ble_rc_foreach_bond_info_cb, &bonded_device_cnt);
        printf("%d peer device(s) bonded\r\n", bonded_device_cnt);
        
        if(ble_rc_start_adv() == 0)
        {
            printf("ble advertising is started\r\n");
        }

        ble_rc_pds_enable(1);
    }
}

void ble_stack_start(void)
{
    // Initialize BLE controller
    btble_controller_init(configMAX_PRIORITIES - 1);
    #if defined(CFG_BLE_PDS)
    btble_controller_sleep_init();
    btble_set_before_sleep_callback(ble_rc_before_sleep_callback);
    btble_set_after_sleep_callback(ble_rc_after_sleep_callback);
    btble_set_sleep_aborted_callback(ble_rc_sleep_aborted_callback);
    #endif
    
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
}
