#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "hosal_gpio.h"
#include "hosal_uart.h"
#include "pds_app.h"

#define APP_UART_TX_PIN       14
#define APP_UART_RX_PIN       15
#define APP_UART_BAUDRATE     2000000

btble_app_conf_t app_conf = 
{
    .print_enable = 1,//1: enable uart print in library; 0: disable uart print in library
    .gpio_irq_restore = 1, //1: restore gpio irq after pds wakeup; 0: do not restore gpio irq after pds wakeup
    .gpio_num = 1, //3,
    .gpio_index = {16, 20, 24},
    .trigger_type = {HOSAL_IRQ_TRIG_NEG_POS_PULSE, HOSAL_IRQ_TRIG_NEG_PULSE, HOSAL_IRQ_TRIG_POS_PULSE},
};

uint8_t wakeup_group4_pin_list[] = {16};
uint8_t wakeup_group4_edge_sel = PDS_GPIO_EDGE_BOTH;
uint8_t wakeup_group5_pin_list[] = {20};
uint8_t wakeup_group5_edge_sel = PDS_GPIO_EDGE_FALLING;
uint8_t wakeup_group6_pin_list[] = {24};
uint8_t wakeup_group6_edge_sel = PDS_GPIO_EDGE_RISING;

void pdsapp_gpio_handler(void *arg)
{
    if(xPortIsInsideInterrupt())
    {
        printf("gpio%d interrupt\r\n", *(uint8_t *)arg);
    }
    else
    {
        printf("gpio%d wakeup\r\n", *(uint8_t *)arg);
    }
}

/* only need to call once, will restore after pds wakeup if app_conf.gpio_irq_restore = 1 */
void pdsapp_gpio_irq_init(void)
{
    hosal_gpio_dev_t key;

    for(int i=0; i<app_conf.gpio_num; i++)
    {
        key.port = app_conf.gpio_index[i];
        key.config = INPUT_PULL_DOWN;
        hosal_gpio_init(&key);
        hosal_gpio_irq_set(&key, app_conf.trigger_type[i], pdsapp_gpio_handler, &app_conf.gpio_index[i]);
    }
}

/* only need to call once */
void pdsapp_gpio_wakeup_init(void)
{
    // use bl_pds_gpio_group_wakeup_cfg or bl_pds_gpio_wakeup_cfg to configure wakeup pins
    // for bl_pds_gpio_group_wakeup_cfg, pins in same group share same wakeup edge, while in different groups can use different wakeup edge
    // for bl_pds_gpio_wakeup_cfg, all pins use same wakeup edge
    bl_pds_gpio_group_wakeup_cfg(4, wakeup_group4_pin_list, sizeof(wakeup_group4_pin_list), wakeup_group4_edge_sel);
    //bl_pds_gpio_group_wakeup_cfg(5, wakeup_group5_pin_list, sizeof(wakeup_group5_pin_list), wakeup_group5_edge_sel);
    //bl_pds_gpio_group_wakeup_cfg(6, wakeup_group6_pin_list, sizeof(wakeup_group6_pin_list), wakeup_group6_edge_sel);

#if !defined(CFG_CLI_DISABLE)
    // configure uart rx pin as pds wakeup pin, so that pds can be wakeup through a cli command (command length should be more than 10 bytes).
    // this command is only for wakeup and can not be well recognized and executed, so better use an unsupported command such as "UUUUUUUUUUUU\r".
    uint8_t pin = APP_UART_RX_PIN;
    uint8_t group = bl_pds_get_gpio_group(pin);
    bl_pds_gpio_group_wakeup_cfg(group, &pin, 1, PDS_GPIO_EDGE_BOTH);
#endif
}

/*
#1.When the device prepares to go into sleep mode, pdsapp_before_sleep_callback will be called. 
#2.If ble sleep preparation is aborted before sleep, pdsapp_sleep_aborted_callback will be called.
#3.After the device wakes up, pdsapp_after_sleep_callback will be called.
*/
int pdsapp_before_sleep_callback(void)
{
    return 0;
}

void pdsapp_sleep_aborted_callback(void)
{
}

void pdsapp_after_sleep_callback(void)
{
    /*if user want to do something after wakeup, add code here.*/
    uint32_t wakeupGpioPinMask = 0;
    uint8_t gpio_idx = 0;

#if defined(CFG_CLI_DISABLE)
    HOSAL_UART_DEV_DECL(uart_stdio, 0, APP_UART_TX_PIN, APP_UART_RX_PIN, APP_UART_BAUDRATE);
    hosal_uart_init(&uart_stdio);
#else
    extern void vfs_uart_restore(void);
    vfs_uart_restore();
#endif
    printf("[%lu]wakeup\r\n", (uint32_t)rom_hal_mtimer_now_us64() / 1000);
    if(bl_pds_get_wakeup_source() == PDS_WAKEUP_BY_GPIO)
    {
        wakeupGpioPinMask = bl_pds_get_wakeup_gpio();
        gpio_idx = __builtin_ctz(wakeupGpioPinMask);
        printf("pds wakeup by gpio,gpio pin mask=0x%lx,gpio_idx=%d\r\n", wakeupGpioPinMask,gpio_idx);
        pdsapp_gpio_handler(&gpio_idx);

#if !defined(CFG_CLI_DISABLE)
        if(gpio_idx == APP_UART_RX_PIN){
            btble_pds_enable(0);
            printf("pds disabled and cli restored\r\n");
            printf("use cli cmd pds_start to enable pds again\r\n");
        }
#endif
    }
}

void pdsapp_init(void)
{
    btble_pds_init(&app_conf);

    pdsapp_gpio_irq_init();

    pdsapp_gpio_wakeup_init();

    btble_pds_enable(1);
}

