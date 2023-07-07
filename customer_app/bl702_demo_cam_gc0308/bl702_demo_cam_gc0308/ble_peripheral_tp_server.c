/****************************************************************************
FILE NAME
    ble_peripheral_tp_server.c

DESCRIPTION
    test profile demo

NOTES
*/
/****************************************************************************/
#if defined(CFG_BLE_ENABLE)
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "bluetooth.h"
#include "conn.h"
#include "gatt.h"
#include "hci_core.h"
#include "uuid.h"
#include "ble_peripheral_tp_server.h"
#include "log.h"
#include <bl_cam.h>
#include "bl702_mjpeg.h"


static void ble_tp_connected(struct bt_conn *conn, u8_t err);
static void ble_tp_disconnected(struct bt_conn *conn, u8_t reason);
static void ble_param_updated(struct bt_conn *conn, u16_t interval,u16_t latency, u16_t timeout);

struct bt_gatt_attr *get_tpattr(u8_t index);
static struct bt_conn *ble_tp_conn;
static struct bt_gatt_exchange_params exchg_mtu;
static TaskHandle_t ble_tp_task_h;

//static struct k_sem notify_poll_sem;

static int tx_mtu_size = 20;
static u8_t created_tp_task = 0;
static u8_t isRegister = 0;

static struct bt_conn_cb ble_tp_conn_callbacks = {
    .connected = ble_tp_connected,
    .disconnected = ble_tp_disconnected,
    .le_param_updated = ble_param_updated,
};

/*************************************************************************
NAME
    ble_tp_tx_mtu_size
*/
static void ble_tp_tx_mtu_size(struct bt_conn *conn, u8_t err,
                               struct bt_gatt_exchange_params *params)
{
    if (!err) {
        tx_mtu_size = bt_gatt_get_mtu(ble_tp_conn);
        BT_WARN("ble tp echange mtu size success, mtu size: %d", tx_mtu_size);
    } else {
        BT_WARN("ble tp echange mtu size failure, err: %d", err);
    }
}

/*************************************************************************
NAME
    ble_tp_connected
*/
static void ble_tp_connected(struct bt_conn *conn, u8_t err)
{
    int tx_octets = 0x00fb;
    int tx_time = 0x0848;
    int ret = -1;

    if (err) {
        return;
    }

    BT_WARN("Tp connected");
    ble_tp_conn = conn;
    
    ret = hci_le_set_phy(ble_tp_conn, 0U, BT_HCI_LE_PHY_PREFER_2M,
				    BT_HCI_LE_PHY_PREFER_2M,
				    BT_HCI_LE_PHY_CODED_ANY);
    if(!ret){
        BT_WARN("Set ble 2M Phy pending \r\n");
    }else{
        BT_WARN("Failed to set ble 2M Phy\r\n");
    }
    //set data length after connected.
    ret = bt_le_set_data_len(ble_tp_conn, tx_octets, tx_time);

    if (!ret) {
        BT_WARN("ble tp set data length success");
    } else {
        BT_WARN("ble tp set data length failure, err: %d", ret);
    }

    //exchange mtu size after connected.
    exchg_mtu.func = ble_tp_tx_mtu_size;
    ret = bt_gatt_exchange_mtu(ble_tp_conn, &exchg_mtu);

    if (!ret) {
        BT_WARN("ble tp exchange mtu size pending");
    } else {
        BT_WARN("ble tp exchange mtu size failure, err: %d", ret);
    }

    struct bt_le_conn_param param;
    param.interval_min = param.interval_max = 12;
    param.latency = 0;
    param.timeout = 0x05dc;
    ret = bt_conn_le_param_update(ble_tp_conn, &param);
    if (ret) {
    BT_WARN("conn update failed (err %d)\r\n", ret);
    } else {
    BT_WARN("conn update initiated\r\n");
    }
}

/*************************************************************************
NAME
    ble_tp_disconnected
*/
static void ble_tp_disconnected(struct bt_conn *conn, u8_t reason)
{
    BT_WARN("Tp disconnected");

    if (created_tp_task) {
        BT_WARN("Delete throughput tx task");
        vTaskDelete(ble_tp_task_h);
        created_tp_task = 0;
    }

    ble_tp_conn = NULL;
}

/*************************************************************************
NAME
    ble_param_updated
*/

static void ble_param_updated(struct bt_conn *conn, u16_t interval,
			     u16_t latency, u16_t timeout)
{

    BT_WARN("LE conn param updated: int 0x%04x lat %d to %d \r\n", interval, latency, timeout);	

    if(interval!=12)
    {
        struct bt_le_conn_param param;
        int err;
        param.interval_min = param.interval_max = 12;
        param.latency = 0;
        param.timeout = 0x05dc;
        err = bt_conn_le_param_update(ble_tp_conn, &param);
        if (err) {
        BT_WARN("conn update failed (err %d)\r\n", err);
        } else {
        BT_WARN("conn update initiated\r\n");
        }
    }

  
        
}

/*************************************************************************
NAME
    ble_tp_recv_rd
*/
static int ble_tp_recv_rd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, u16_t len, u16_t offset)
{
    int size = 9;
    char data[9] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09 };

    memcpy(buf, data, size);

    return size;
}

/*************************************************************************
NAME
    ble_tp_recv_wr(receive data from client)
*/
static int ble_tp_recv_wr(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, u16_t len, u16_t offset, u8_t flags)
{
    BT_WARN("recv data len=%d, offset=%d, flag=%d", len, offset, flags);
    BT_WARN("recv data:%s", bt_hex(buf, len));

    if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
        //Don't use prepare write data, execute write will upload data again.
        BT_WARN("recv prepare write request");
        return 0;
    }

    if (flags & BT_GATT_WRITE_FLAG_CMD) {
        //Use write command data.
        BT_WARN("recv write command");
    } else {
        //Use write request / execute write data.
        BT_WARN("recv write request / exce write");
    }

    return len;
}

/*************************************************************************
NAME
    indicate_rsp /bl_tp_send_indicate
*/

static void indicate_rsp(struct bt_conn *conn, const struct bt_gatt_attr *attr, u8_t err)
{
    BT_WARN("receive comfirmation, err:%d", err);
}

static int bl_tp_send_indicate(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               const void *data, u16_t len)
{
    static struct bt_gatt_indicate_params ind_params;

    ind_params.attr = attr;
    ind_params.data = data;
    ind_params.len = len;
    ind_params.func = indicate_rsp;
    ind_params.uuid = NULL;

    return bt_gatt_indicate(conn, &ind_params);
}

/*************************************************************************
NAME
    ble_tp_ind_ccc_changed
*/
static void ble_tp_ind_ccc_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    int err = -1;
    char data[9] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09 };

    if (value == BT_GATT_CCC_INDICATE) {
        err = bl_tp_send_indicate(ble_tp_conn, get_tpattr(BT_CHAR_BLE_TP_IND_ATTR_VAL_INDEX), data, 9);
        BT_WARN("ble tp send indatcate: %d", err);
    }
}

#define BLE_PACKET_LENGTH 244
static void ble_tp_notify_task(void *pvParameters)
{
    int len;
    int8_t *ptr1, *ptr2;
    static uint32_t counter = 0;
    uint32_t len1 = 0, len2 = 0, frames;
    int err = -1;
    static uint16_t cnt = 0;
    static uint64_t time_last;
    float cam_fps = 0;
    bl_cam_frame_pop_old();
    time_last = xTaskGetTickCount();

    printf("now ms =%ld\r\n",xTaskGetTickCount());
    while(1)
    {   
        if (0 == bl_cam_frame_get(&frames, &ptr1, &len1, &ptr2, &len2)) {
            printf("[YUV] %d frames %u, frame ptr1 %p len %u, ptr2 %p len %u; counter %u\r\n",
                (int)xTaskGetTickCount(),
                (unsigned int)frames,
                ptr1,
                (unsigned int)len1,
                ptr2,
                (unsigned int)len2,
                (unsigned int)counter++
            );
            MJPEG_Disable();

            ptr2 = ptr1;

            while(len1 > 0) 
            {
                len = len1 >= BLE_PACKET_LENGTH ? BLE_PACKET_LENGTH : len1;
                err = bt_gatt_notify(ble_tp_conn, get_tpattr(BT_CHAR_BLE_TP_NOT_ATTR_VAL_INDEX), ptr2, BLE_PACKET_LENGTH);
                if(err==0)
                {
                    ptr2 += len;
                    len1 -= len;
                }
            }
            MJPEG_Enable();
            cnt++;
            if (cnt >= 10) {
                uint64_t time = xTaskGetTickCount();
                cam_fps = (1000 * 10) / (float)(time - time_last);
                time_last = time;
                cnt = 0;
                printf("cam_fps %.1f\r\n", cam_fps);
            }
        }
        vTaskDelay(10);
    }
}

/*************************************************************************
NAME
    ble_tp_not_ccc_changed
*/
static void ble_tp_notify_ccc_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    BT_WARN("ccc:value=[%d]", value);

    if (value == BT_GATT_CCC_NOTIFY) {

      
        if (xTaskCreate(ble_tp_notify_task, (char *)"bletp", 512, NULL, 15, &ble_tp_task_h) == pdPASS) {
            created_tp_task = 1;
            BT_WARN("Create throughput tx task success");
        } else {
            created_tp_task = 0;
            BT_WARN("Create throughput tx taskfail");
        }
    } else {
        if (created_tp_task) {
            BT_WARN("Delete throughput tx task");
            vTaskDelete(ble_tp_task_h);
            created_tp_task = 0;
        }
        printf("now ms =%ld\r\n",xTaskGetTickCount());
         
    }
}

/*************************************************************************
*  DEFINE : attrs
*/
static struct bt_gatt_attr attrs[] = {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SVC_BLE_TP),

    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_BLE_TP_RD,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           ble_tp_recv_rd,
                           NULL,
                           NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_BLE_TP_WR,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE | BT_GATT_PERM_PREPARE_WRITE,
                           NULL,
                           ble_tp_recv_wr,
                           NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_BLE_TP_IND,
                           BT_GATT_CHRC_INDICATE,
                           0,
                           NULL,
                           NULL,
                           NULL),

    BT_GATT_CCC(ble_tp_ind_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_CHAR_BLE_TP_NOT,
                           BT_GATT_CHRC_NOTIFY,
                           0,
                           NULL,
                           NULL,
                           NULL),

    BT_GATT_CCC(ble_tp_notify_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

};

/*************************************************************************
NAME
    get_tpattr
*/
struct bt_gatt_attr *get_tpattr(u8_t index)
{
    return &attrs[index];
}

static struct bt_gatt_service ble_tp_server = BT_GATT_SERVICE(attrs);

/*************************************************************************
NAME
    ble_tp_init
*/
void ble_cam_init(void)
{
    if (!isRegister) {
        isRegister = 1;
        bt_conn_cb_register(&ble_tp_conn_callbacks);
        bt_gatt_service_register(&ble_tp_server);
    }
}
#endif