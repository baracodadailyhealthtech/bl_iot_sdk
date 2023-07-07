#if defined(CFG_BLE_ENABLE)
#include <stdint.h>
#include "bl_irq.h"
#include "ble_lib_api.h"
#include "bluetooth.h"
#include "hci_driver.h"
#include "hci_core.h"
#include "log.h"
#include "ble_app.h"
#if defined(CFG_BLE_ENABLE_STACK_CLI) 
#include "ble_cli_cmds.h"
#endif


#define BLE_APP_DEV_NAME "BL702_BLE"
#define BLE_APP_ADV_INT_MIN 0x03c0
#define BLE_APP_ADV_INT_MAX 0x03c0

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

extern void ble_cam_init(void);
void bt_enable_cb(int err)
{
    if (!err) {
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);

        hci_le_set_default_phy(BT_HCI_LE_PHY_PREFER_2M);
        #if defined(CONFIG_BT_STACK_CLI) 
        ble_cli_register();
        #endif
        ble_cam_init();
        bt_set_name(BLE_APP_DEV_NAME);
        if(bleapp_start_adv() == 0)
        {
            printf("ble advertising is started\r\n");
        }
    }
    else
    {
         printf("bt_enable_cb err\r\n");
    }
}

void ble_stack_start(void)
{
    // Initialize BLE controller
    ble_controller_init(configMAX_PRIORITIES - 1);
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
}
#endif