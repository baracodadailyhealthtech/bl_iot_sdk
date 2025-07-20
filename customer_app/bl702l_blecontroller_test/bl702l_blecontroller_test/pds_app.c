#if defined (CFG_PDS_ENABLE)
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "hosal_gpio.h"
#include "hosal_uart.h"
#include "pds_app.h"
#if defined(CONFIG_LP_HCIUART)
#include "hci_uart_lp.h"
#endif
#include "bl_wdt.h"

btble_app_conf_t app_conf = 
{
    .print_enable = 1,//1: enable uart print in library; 0: disable uart print in library
    .gpio_irq_restore = 0, //1: restore gpio irq after pds wakeup; 0: do not restore gpio irq after pds wakeup
    .gpio_num = 0,
    .gpio_index = {},
    .pull_type = {},
    .trigger_type = {},
};

extern uint32_t bl_timer_now_us(void);

/* only need to call once */
void pdsapp_gpio_wakeup_init(void)
{
	uint8_t uart_rx_pin = 15;

    bl_pds_gpio_wakeup_cfg(&uart_rx_pin, 1, PDS_GPIO_EDGE_BOTH);
	//bl_pds_gpio_pull_set(0, 1<<uart_rx_pin);
}

/*
#1.When the device prepares to go into sleep mode, pdsapp_before_sleep_callback will be called. 
#2.If ble sleep preparation is aborted before sleep, pdsapp_sleep_aborted_callback will be called.
#3.After the device wakes up, pdsapp_after_sleep_callback will be called.
*/
int pdsapp_before_sleep_callback(void)
{ 
    #if defined(CONFIG_LP_HCIUART)
    if (hci_uart_is_busy())
    {
        return -1;
    }
    #endif
        
    return 0;
}

void pdsapp_sleep_aborted_callback(void)
{
}

void pdsapp_after_sleep_callback(void)
{
    // only wakeup by UART rx pin keep active some time
    if (bl_pds_get_wakeup_source() == PDS_WAKEUP_BY_GPIO)
        hci_uart_wakeup();

    /*deinit hosal dma before calling reinit APIs based on hosal dma*/
    hosal_dma_finalize();

#if !defined(GPIO_SIM_PRINT)
    HOSAL_UART_DEV_DECL(uart_stdio, 0, APP_UART_TX_PIN, APP_UART_RX_PIN, APP_UART_BAUDRATE);
    hosal_uart_init(&uart_stdio);
#else
	extern int bl_gpio_uart_tx_init(uint8_t id, uint8_t tx_pin, uint32_t baudrate);
    bl_gpio_uart_tx_init(0, GPIO_SIM_PRINT_TX_PIN, GPIO_SIM_PRINT_BAUDRATE);
#endif
    printf("[%lu] pds wake\r\n", bl_timer_now_us());
    #if defined(CONFIG_LP_HCIUART)
	hci_uart_init(14,15);
    #endif
	bl_pds_gpio_pull_disable();

    bl_wdt_init(30 * 1000);
}

#if !defined(CFG_USE_ROM_CODE) || defined(CFG_BUILD_FREERTOS)
void vApplicationSleep(TickType_t xExpectedIdleTime)
{
    btble_vApplicationSleepExt(xExpectedIdleTime);
}
#endif

void pdsapp_init(void)
{
    #if defined(CFG_USE_ROM_CODE) && !defined(CFG_BUILD_FREERTOS)
    vApplicationSleep = btble_vApplicationSleepExt;
    #endif

    btble_pds_init(&app_conf);

	pdsapp_gpio_wakeup_init();

    btble_pds_enable(1);
}
#endif

