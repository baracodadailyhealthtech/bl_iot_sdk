/*
 * Copyright (c) 2016-2024 Bouffalolab.
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

#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>
#include <stdbool.h>
/**
 ****************************************************************************************
 * @defgroup UART UART
 * @ingroup DRIVERS
 * @brief UART driver
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/**
 *  @brief UART baudrate configuration
 */
typedef enum
{
  BAUDRATE_DIV_1200 = 26666,
  BAUDRATE_DIV_2400 = 13332,
  BAUDRATE_DIV_9600 = 3333,
  BAUDRATE_DIV_19200 = 1667,
  BAUDRATE_DIV_38400 = 833,
  BAUDRATE_DIV_57600 = 555,
  BAUDRATE_DIV_115200 = 277,
  BAUDRATE_DIV_230400 = 138,
  BAUDRATE_DIV_460800 = 68,
  BAUDRATE_DIV_921600 = 34,
  BAUDRATE_DIV_1792000 = 17,
  BAUDRATE_DIV_1843200 = 16,
  BAUDRATE_DIV_3000000 = 10,
  BAUDRATE_DIV_3584000 = 8,
  BAUDRATE_DIV_3686400 = 8,
  BAUDRATE_DIV_7168000 = 3,
  BAUDRATE_DIV_7372800 = 3,
  BAUDRATE_DIV_9000000 = 3,
  BAUDRATE_DIV_10500000 = 2
}UART_BAUDRATE_DIV_CPU32;

typedef enum
{
  TX_FIFO_DISABLE = 0,/*UART TX FIFO interrupt is disabled*/
  TX_FIFO_EMPTY,      /*UART TX FIFO interrrupt is triggered when TX FIFO is empty*/
  TX_FIFO_CNT_32      /*UART TX FIFO interrrupt is triggered when TX FIFO  count reaches 32*/
}UART_TX_FIFO_ISR_TYPE;

typedef enum
{
  RX_FIFO_DISABLE = 0,       /*UART RX FIFO interrupt is disabled*/
  RX_FIFO_NOT_EMPTY = 1,     /*UART RX FIFO interrrupt is triggered when RX FIFO is not empty*/
  RX_FIFO_CNT_8 = 2,         /*UART RX FIFO interrrupt is triggered when RX FIFO count reaches 8*/
  RX_FIFO_CNT_16 = 4,        /*UART RX FIFO interrrupt is triggered when RX FIFO count reaches 16*/
  RX_FIFO_CNT_32 = 8         /*UART RX FIFO interrrupt is triggered when RX FIFO count reaches 32*/
}UART_RX_FIFO_ISR_TYPE;

/**
 * @brief Get baudrate divisor macro
 */ 
#define uart_get_baudrate_div(baudrate) (BAUDRATE_DIV_##baudrate)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the UART gpio.
 *****************************************************************************************
 */
void btble_uart_pin_config(uint8_t uartid, uint8_t tx, uint8_t rx, uint8_t cts, uint8_t rts);

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */
void btble_uart_init(uint8_t uartid);

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */
int8_t btble_uart_reconfig(uint32_t baudrate, uint8_t flow_ctl_en, uint8_t cts_pin, uint8_t rts_pin);

/**
 ****************************************************************************************
 * @brief Enable UART flow.
 *****************************************************************************************
 */
void btble_uart_flow_on(void);

/**
 ****************************************************************************************
 * @brief Disable UART flow.
 *****************************************************************************************
 */
bool btble_uart_flow_off(void);

/**
 ****************************************************************************************
 * @brief Finish current UART transfers
 *****************************************************************************************
 */
void btble_uart_finish_transfers(void);

/**
 ****************************************************************************************
 * @brief Starts a data reception.
 *
 * @param[out] bufptr   Pointer to the RX buffer
 * @param[in]  size     Size of the expected reception
 * @param[in]  callback Pointer to the function called back when transfer finished
 * @param[in]  dummy    Dummy data pointer returned to callback when reception is finished
 *****************************************************************************************
 */
void btble_uart_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data transmission.
 *
 * @param[in] bufptr   Pointer to the TX buffer
 * @param[in] size     Size of the transmission
 * @param[in] callback Pointer to the function called back when transfer finished
 * @param[in] dummy    Dummy data pointer returned to callback when transmission is finished
 *****************************************************************************************
 */
void btble_uart_write(const uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

#endif /* _UART_H_ */
