#ifndef __BLE_SYNC_H_
#define __BLE_SYNC_H_

void blsync_ble_start (void);

void blsync_ble_stop (void);

void blesync_wifi_scan_done(netbus_fs_scan_ind_cmd_msg_t *msg);

#endif 
