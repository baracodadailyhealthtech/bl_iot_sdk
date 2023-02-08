#ifndef __BLE_RC_APP_H__
#define __BLE_RC_APP_H__

typedef enum{
    RC_KYS_NOTIFY,
    RC_KYS_UNPAIR,
    RC_KYS_VOICE_START,
    RC_KYS_VOICE_STOP,
}ble_rc_kys_evt_type;

void ble_stack_start(void);
#endif //__BLE_RC_APP_H__