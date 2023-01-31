/*
 * Copyright (c) 2016-2023 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef BLE_LIB_API_H_
#define BLE_LIB_API_H_

#include <stdbool.h>
#include <stdint.h> 


#if defined(CONFIG_BLE_MFG)
#define MAX_SWITCHING_PATTERN_LEN  (0x4B)
struct hci_le_rx_test_v2_cmd
{
    uint8_t     rx_channel;
    uint8_t     phy;
    uint8_t     mod_idx;
};

struct hci_le_tx_test_v4_cmd
{
    uint8_t     tx_channel;
    uint8_t     test_data_len;
    uint8_t     pkt_payl;
    uint8_t     phy;
    uint8_t     cte_len;
    uint8_t     cte_type;
    uint8_t     switching_pattern_len;
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
    int8_t     tx_pwr_lvl;
};
#endif

#if defined(CONFIG_BT_MFG)
struct hci_vs_rx_test_cmd
{
    uint8_t     rx_channel;
    uint8_t     pkt_type;
};
struct hci_vs_tx_test_cmd
{
    uint8_t     tx_channel;
    uint16_t     test_data_len;
    uint8_t     pkt_payl;
    uint8_t     pkt_type;
    int8_t     tx_pwr_lvl;
};
#endif

void btble_controller_init(uint8_t task_priority);
void ble_controller_deinit(void);

extern int32_t btble_controller_sleep(int32_t max_sleep_cycles);
extern void btble_controller_sleep_restore();
#if defined(CFG_BT_RESET)
void ble_controller_reset(void);
#endif

// return sleep duration, in unit of 1/32768s
// if 0, means not allow sleep
// if -1, means allow sleep, but there is no end of sleep interrupt (ble core deep sleep is not enabled)
bool ble_controller_sleep_is_ongoing(void);

char *btble_controller_get_lib_ver(void);

void btble_controller_remaining_mem(uint8_t** addr, int* size);
#if defined(BL702L)
void ble_controller_set_tx_pwr(int ble_tx_power);
#endif

#if defined (CONFIG_BLE_MFG) || defined (CONFIG_BT_MFG) 
int bt_mfg_cli_register(void);
int reset_cmd_handler(void);
#if defined (CONFIG_BLE_MFG)
int hci_le_tx_test_v2_cmd_handler(struct hci_le_tx_test_v2_cmd const *param, uint16_t opcode, bool from_hci);
int hci_le_tx_test_v4_cmd_handler(struct hci_le_tx_test_v4_cmd const *param, uint16_t opcode,bool from_hci);
int hci_le_rx_test_v2_cmd_handler(struct hci_le_rx_test_v2_cmd const *param, uint16_t opcode, bool from_hci);
int hci_le_test_end_cmd_handler(void const *param, uint16_t opcode, bool from_hci);
bool ble_check_test_ongoing(void);
#endif
#if defined (CONFIG_BT_MFG)
int hci_vs_rx_test_cmd_handler(struct hci_vs_rx_test_cmd const *param, uint16_t opcode, bool from_hci);
int hci_vs_tx_test_cmd_handler(struct hci_vs_tx_test_cmd const *param, uint16_t opcode, bool from_hci);
int hci_vs_test_end_cmd_handler(void const *param, uint16_t opcode, bool from_hci);
#endif
#endif
#endif
