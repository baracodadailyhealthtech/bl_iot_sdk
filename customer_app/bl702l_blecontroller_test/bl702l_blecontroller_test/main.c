#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <vfs.h>
#include <device/vfs_uart.h>
#include <aos/kernel.h>
#include <aos/yloop.h>
#include <event_device.h>
#include <cli.h>

#include <bl702l_glb.h>
#include <bl_sys.h>
#include <bl_chip.h>
#include <bl_irq.h>
#include <hosal_uart.h>
#include "bl_uart.h"
#include "btble_lib_api.h"
#if defined(CONFIG_LP_HCIUART)
#include "hci_uart_lp.h"
#endif
#if defined(CFG_PDS_ENABLE)
#include "pds_app.h"
#endif
#include "bl_wdt.h"

TaskHandle_t btble_init_task_hdl;

extern void ble_uart_init(uint8_t uartid);
extern volatile bool sys_log_all_enable;

uint32_t btble_get_sdk_ver(void)
{
   char array[9];
   const char *s = BL_SDK_VER;
   uint32_t val;

   while(*s){
       s++;
   }

   s--;
   while(*s != 'g'){
       s--;
   }

   s++;
   memcpy(array, s, 8);
   array[8] = 0;
   sscanf(array, "%lx", &val);
   return val;
}

static void btble_init_task_entry(void *pvParameters)
{
    btble_controller_set_local_sdk_ver(btble_get_sdk_ver());
    btble_controller_init(configMAX_PRIORITIES - 1);
    printf("btble_init_task_entry end\r\n");
    #if defined(CFG_PDS_ENABLE)
    btble_set_before_sleep_callback(pdsapp_before_sleep_callback);
    btble_set_after_sleep_callback(pdsapp_after_sleep_callback);
    btble_set_sleep_aborted_callback(pdsapp_sleep_aborted_callback);
    #endif
    vTaskDelete(NULL);
}

void main()
{
    //sys_log_all_enable = false;
    #if defined(CONFIG_LP_HCIUART)
    hci_uart_init(14,15);
    #else
    ble_uart_init(0);
    #endif
    #if defined(CFG_PDS_ENABLE)
    printf("pds_init\r\n");
    pdsapp_init();
    #endif

    bl_wdt_init(30 * 1000); // enable watchdog, timeout is 30 seconds

    HBN_Set_User_Boot_Config(2); // enter flash boot mode after sw reset

    puts("creat btble_init_task_entry\r\n");
    xTaskCreate(btble_init_task_entry, (char*)"bleinit", 2*1024, NULL, 15, &btble_init_task_hdl);
}
