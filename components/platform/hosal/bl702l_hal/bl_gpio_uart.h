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
#ifndef __BL_GPIO_UART_H__
#define __BL_GPIO_UART_H__

#include <stdint.h>
#include <math.h>

#define GPIO_UART_TX_NUM           1

// tx_pin: 0 - 31
// baudrate: max 2Mbps @128M or 1Mbps @32M
int bl_gpio_uart_tx_init(uint8_t id, uint8_t tx_pin, uint32_t baudrate);
int bl_gpio_uart_send_byte(uint8_t id, uint8_t data);
int bl_gpio_uart_send_data(uint8_t id, uint8_t *data, uint32_t len);

// rx_pin: 0 - 31, two rx pins (connected together) are required
// baudrate: max 9600bps
int bl_gpio_uart_rx_init(uint8_t rx_pin_1, uint8_t rx_pin_2, uint32_t baudrate, uint32_t fifo_size);
uint32_t bl_gpio_uart_get_rx_length(void);
uint32_t bl_gpio_uart_read_data(uint8_t *data, uint32_t len);

#endif
