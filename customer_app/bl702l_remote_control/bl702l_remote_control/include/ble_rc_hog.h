#ifndef __BLE_RC_HOG_H__
#define __BLE_RC_HOG_H__

#include <types.h>
#define HID_PAGE_KBD   0x07
#define HID_PAGE_CONS  0x0C

struct hids_remote_key {
    u8_t hid_page;
    u8_t *hid_usage;
} __packed;

void hog_init(void);
extern volatile u8_t Voicekey_is_press;
int bt_hog_notify(struct bt_conn *conn, u8_t hid_page, u8_t *hid_usage, bool press);
struct bt_gatt_attr *ble_rc_get_voice_attr(void);
#endif //__BLE_RC_HOG_H__