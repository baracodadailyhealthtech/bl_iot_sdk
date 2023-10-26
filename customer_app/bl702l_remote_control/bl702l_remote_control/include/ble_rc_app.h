#ifndef __BLE_RC_APP_H__
#define __BLE_RC_APP_H__

#define BLE_RC_NAME "BL702L_RC"
#define BLE_RC_DEVICE_APPEARANCE 0x03C1
#define BLE_RC_ADV_TIMER_IN_SEC 5 //5s
#define BLE_RC_UNPAIR_TIMER_IN_SEC 15 //15s
#define BLE_RC_ADC_TIMER_IN_SEC 60 //60s

typedef enum{
    RC_KYS_IR_TX,
    RC_KYS_NOTIFY,
    RC_KYS_ADV,
    RC_KYS_VOICE_START,
    RC_KYS_VOICE_STOP,
    RC_KYS_DELETE_ADV_TIMER,
    RC_KYS_REPORT_BATTERY_LEVEL,
    RC_KYS_INVALID_EVT = 0xFF
}ble_rc_kys_evt_type;

void ble_stack_start(void);
void ble_rc_pds_enable(uint8_t enable);
void ble_rc_kys_init(void);
void ble_rc_check_pending_evt(void);
int ble_rc_connection_update(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);
#endif //__BLE_RC_APP_H__