/*
 * Copyright (c) 2016-2025 Bouffalolab.
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
#ifndef __HCI_UART_LP_H__
#define __HCI_UART_LP_H__


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


#define HCI_UART_OK                 0
#define HCI_UART_INIT_FAULT         -1
#define HCI_UART_TX_QUEUE_FULL      -2
#define HCI_UART_TX_FAULT           -3
#define HCI_UART_WAKEUP_FAULT       -4
#define HCI_UART_WAKEUP_TIMEOUT     -5


int hci_uart_init(uint8_t tx_pin, uint8_t rx_pin);
int hci_uart_write(uint8_t *data, uint32_t len);
uint32_t hci_uart_get_rx_count(void);
uint32_t hci_uart_read(uint8_t *data, uint32_t len);
bool hci_uart_is_busy(void);
void hci_uart_wakeup(void);

void hci_uart_ack_received_callback(void);  // inside task context
void hci_uart_rx_done_callback(uint8_t *data, uint32_t len);  // inside interrupt context


#endif
