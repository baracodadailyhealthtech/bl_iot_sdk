/** @file
 *  @brief HOGP Service sample
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include "byteorder.h"
#include "bluetooth.h"
#include "hci_host.h"
#include "conn.h"
#include "uuid.h"
#include "gatt.h"
#include "log.h"
#include "ble_rc_hog.h"
#include "ble_rc_voice.h"

u16_t BATTERY_LEVEL_UUID_VAL = 0x2a19;
#define BT_GATT_HIDS_EXT_REPORT(_uuid)  \
    BT_GATT_ATTRIBUTE(BT_UUID_HIDS_EXT_REPORT, BT_GATT_PERM_READ, \
    read_ext_report_ref, NULL, _uuid)

enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    u16_t version; /* version number of base USB HID Specification */
    u8_t code; /* country HID Device hardware is localized for. */
    u8_t flags;
} __packed;

struct hids_report {
    u8_t id; /* report id */
    u8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .version = 256,
    .code = 0x00,
    .flags = 0,
};

enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

#define HID_REPORT_REGISTER(_id, _type)		\
{              \
    .id = _id, \
    .type =_type, \
}

static struct hids_report report[3]= {
    /*input_1 */	
    HID_REPORT_REGISTER(0x01,HIDS_INPUT),
    /*input_2 */
    HID_REPORT_REGISTER(0x02,HIDS_INPUT),
    /*input_3 */
    HID_REPORT_REGISTER(0xf0,HIDS_INPUT),
};

static bool simulate_input;
static u8_t ctrl_point;
static u8_t report_map[] = {
    /*
     * Keyboard 
     */      
    0x05, 0x01,     //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,     //   Usage (Keyboard)
    0xA1, 0x01,     //   Collection (Application)
    0x85, 0x01,     //   Report Id (1) 
    0x05, 0x07,     //   USAGE_PAGE (Keyboard)
    0x19, 0xE0,     //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xE7,     //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,     //   LOGICAL_MINIMUM (0)
    0x25, 0x01,     //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,     //   REPORT_SIZE (1)
    0x95, 0x08,     //   REPORT_COUNT (8)
    0x81, 0x02,     //   INPUT (Data,Var,Abs)
    0x95, 0x01,     //   REPORT_COUNT (1)
    0x75, 0x08,     //   REPORT_SIZE (8)
    0x81, 0x01,     //   INPUT (Cnst,Ary,Abs)
    0x95, 0x05,     //   REPORT_COUNT (5)
    0x75, 0x01,     //   REPORT_SIZE (1)
    0x05, 0x08,     //   USAGE_PAGE (LEDs)
    0x19, 0x01,     //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,     //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,     //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,     //   REPORT_COUNT (1)
    0x75, 0x03,     //   REPORT_SIZE (3)
    0x91, 0x01,     //   OUTPUT (Cnst,Ary,Abs)
    0x95, 0x06,     //   REPORT_COUNT (6)
    0x75, 0x08,     //   REPORT_SIZE (8)
    0x15, 0x00,     //   LOGICAL_MINIMUM (0)
    0x25, 0xFF,     //   LOGICAL_MAXIMUM (255)
    0x05, 0x07,     //   USAGE_PAGE (Keyboard)
    0x19, 0x00,     //   USAGE_MINIMUM (0x00)
    0x29, 0xFF,     //   USAGE_MAXIMUM (0xFF)
    0x81, 0x00,     //   INPUT (Data,Ary,Abs)
    0xC0,           // END_COLLECTION
    
    /*
    *  Consumer Control
    */
    0x05, 0x0C,         //   Usage Page (Consumer)
    0x09, 0x01,         //   Usage (Consumer Control)
    0xA1, 0x01,         //   Collection (Application) 
    0x85, 0x02,         //   Report Id (2)
    0x19, 0x00,         //   USAGE_MINIMUM (0)
    0x2A, 0xff, 0x03,   //   USAGE_MAXIMUM (0x29C)
    0x15, 0x00,         //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x03,   //   LOGICAL_MAXIMUM (0x29C)
    0x95, 0x01,         //   REPORT_COUNT (1)
    0x75, 0x10,         //   REPORT_SIZE (16)
    0x81, 0x00,         //   INPUT (Data,Ary,Abs)
    0xC0,               // END_COLLECTION

    /*
    *  Vendor Defined
    */
    0x06, 0x00, 0xFF, /* Usage Page (Vendor-defined 0xFF00) */
    0x09, 0x00, /* Usage (Vendor-defined 0x0000) */
    0xA1, 0x01, /* Collection (Application) */
    0x85, 0xF0, /*   Report Id (240) */
    0x95, 0x86, /*   Report Count (134) */
    0x75, 0x08, /*   Report Size (8) */
    0x15, 0x00, /*   Logical minimum (0)*/
    0x25, 0xFF, /*   Logical maxmum (255)*/
    0x81, 0x00, /*   Input (Data,Array,Absolute,Bit Field) */
    0xC0,
};

static ssize_t read_info(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, void *buf,
                               u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, void *buf,
                               u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,sizeof(report_map));
}

static ssize_t read_ext_report_ref(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr, void *buf,
                                u16_t len, u16_t offset)       
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(u16_t));
}

static ssize_t read_report(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr, void *buf,
                                u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,sizeof(struct hids_report));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    ARG_UNUSED(attr);
    simulate_input = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_input_report(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, void *buf,
                                 u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}


static ssize_t write_ctrl_point(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, u16_t len, u16_t offset,
                                 u8_t flags)
{
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(ctrl_point)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	return len;
}

/*
*  gatt attribute list,
*  
*  note : Macro definition include several gatt attribute 
*  eg : attrs[2] : BT_GATT_INCLUDE_SERVICE(x), 
*         attrs[3] \attrs[4] : BT_GATT_CHARACTERISTIC
*/   
static struct bt_gatt_attr attrs[]= 
{
    /*primary server uuid*/
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, 
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_info,
                           NULL, 
                           &info),
    
    /*
    * control point characteristic 
    */	
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, 
                           write_ctrl_point, 
                           &ctrl_point),
       
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, 
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_report_map, 
                           NULL, 
                           NULL),
    
    BT_GATT_HIDS_EXT_REPORT(&BATTERY_LEVEL_UUID_VAL),
    /*
    * report characteristic 
    * descriptor declaration "input"
    */	
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_AUTHEN,
                           read_input_report, 
                           NULL, 
                           NULL),
    
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, 
                       BT_GATT_PERM_READ,
                       read_report, 
                       NULL, 
                       &report[0]),
    /*
    * report characteristic 
    * descriptor declaration "input_2"
    */   
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_AUTHEN,
                           read_input_report, 
                           NULL,
                           NULL),
    
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
     
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, 
                       BT_GATT_PERM_READ,
                       read_report,
                       NULL, 
                       &report[1]),

    /*
    * report characteristic  
    * descriptor declaration "input_3"
    */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_AUTHEN,
                           read_input_report,
                           NULL, 
                           NULL),

    BT_GATT_CCC(input_ccc_changed,BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
    
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, 
                       BT_GATT_PERM_READ,
                       read_report, 
                       NULL, 
                       &report[2]),
};

static struct bt_gatt_service hog_svc = BT_GATT_SERVICE(attrs);

int bt_hog_notify(struct bt_conn *conn, u8_t hid_page, u8_t *hid_usage, bool press)
{
	struct hids_remote_key *remote_key = NULL;
    struct bt_gatt_attr *attr = NULL;

	u8_t data[8];
    u8_t usage_size = 2;
    
    printf("conn=%p,hid_page=%d,hid_usage=%p,press=%d\r\n ",conn,hid_page,hid_usage,press);
    if(hid_page == HID_PAGE_KBD)
        usage_size = 8;
          
    for(int i = 0; i < (sizeof(remote_kbd_map_tab)/sizeof(remote_kbd_map_tab[0])); i++){ 
        if(remote_kbd_map_tab[i].hid_page == hid_page){
            if(!memcmp(hid_usage, remote_kbd_map_tab[i].hid_usage, usage_size))
            {
                remote_key = (struct hids_remote_key *)&remote_kbd_map_tab[i];
                break;
            }
        }
    }
    
    if(!remote_key)
        return EINVAL;
    
    if(remote_key->hid_page == HID_PAGE_KBD){
        attr = &attrs[9];
    
    }else if(remote_key->hid_page == HID_PAGE_CONS){
        attr = &attrs[13];
    }else
    return EINVAL;	
     
    if(!press){
        memset(data, 0, usage_size);
        return bt_gatt_notify(conn, attr, data, usage_size);
    }
    memcpy(data, remote_key->hid_usage, usage_size);
    return bt_gatt_notify(conn, attr, data, usage_size);
}

struct bt_gatt_attr *ble_rc_get_voice_attr(void)
{
   return &attrs[17];
}

void hog_init(void)
{
	bt_gatt_service_register(&hog_svc);
    ble_rc_voice_cfg();
}
