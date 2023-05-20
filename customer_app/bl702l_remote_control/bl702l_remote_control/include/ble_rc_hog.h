#ifndef __BLE_RC_HOG_H__
#define __BLE_RC_HOG_H__

#include <types.h>
#define HID_PAGE_KBD   0x07
#define HID_PAGE_CONS  0x0C
static uint8_t KEY_CH_INS[8]   = {0x00,0x00,0x4b,0x00,0x00,0x00,0x00,0x00}; //Keyboard Pageup
static uint8_t KEY_CH_DES[8]   = {0x00,0x00,0x4e,0x00,0x00,0x00,0x00,0x00}; //Keyboard Pagedown
static uint8_t KEY_LEFT[8]     = {0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00}; //keyboard RightArrow
static uint8_t KEY_RIGHT[8]    = {0x00,0x00,0x4f,0x00,0x00,0x00,0x00,0x00}; //keyboard LeftArrow
static uint8_t KEY_DOWN[8]     = {0x00,0x00,0x51,0x00,0x00,0x00,0x00,0x00}; //keyboard DownArrow
static uint8_t KEY_UP[8]       = {0x00,0x00,0x52,0x00,0x00,0x00,0x00,0x00}; //keyboard UpArrow
static uint8_t KEY_MENU[8]     = {0x00,0x00,0x65,0x00,0x00,0x00,0x00,0x00}; //keyboard Application
static uint8_t KEY_PWR[8]      = {0x00,0x00,0x66,0x00,0x00,0x00,0x00,0x00}; //keyboard power
static uint8_t KEY_PICK[2]     = {0x41,0x00}; //Menu Pick  
static uint8_t KEY_MUTE[2]     = {0xe2,0x00}; //MUTE
static uint8_t KEY_VOL_INS[2]  = {0xe9,0x00}; //Volume Increment
static uint8_t KEY_VOL_DES[2]  = {0xea,0x00}; //Volume Decrement
static uint8_t KEY_HOME[2]     = {0x23,0x02}; //AC Home
static uint8_t KEY_BACK[2]     = {0x24,0x02}; //AC Back

struct hids_remote_key {
    u8_t hid_page;
    u8_t *hid_usage;
} __packed;

static struct hids_remote_key remote_kbd_map_tab[] = {
    {HID_PAGE_KBD, KEY_MENU},
    {HID_PAGE_KBD, KEY_CH_INS},
    {HID_PAGE_KBD, KEY_CH_DES},
    {HID_PAGE_CONS, KEY_VOL_INS},
    {HID_PAGE_CONS, KEY_VOL_DES},
    {HID_PAGE_KBD, KEY_UP},
    {HID_PAGE_CONS, KEY_BACK},
    {HID_PAGE_KBD, KEY_PWR},
    {HID_PAGE_KBD, KEY_LEFT},
    {HID_PAGE_CONS, KEY_PICK},
    {HID_PAGE_KBD, KEY_RIGHT},
    {HID_PAGE_CONS, KEY_MUTE},
    {HID_PAGE_CONS, KEY_HOME},
    {HID_PAGE_KBD, KEY_DOWN},
};
    
void hog_init(void);
extern volatile u8_t Voicekey_is_press;
int bt_hog_notify(struct bt_conn *conn, u8_t hid_page, u8_t *hid_usage, bool press);
struct bt_gatt_attr *ble_rc_get_voice_attr(void);
#endif //__BLE_RC_HOG_H__