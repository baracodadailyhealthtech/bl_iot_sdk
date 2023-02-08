#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>

#include <bl_wireless.h>
#include <main.h>

void vApplicationTickHook( void )
{
#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_monitor(void);
    usb_cdc_monitor();
#endif
}

void vApplicationSleep( TickType_t xExpectedIdleTime )
{
    
}

void otrAppProcess(ot_system_event_t sevent) 
{
    /** for application code */
    /** Note,   NO heavy execution, no delay and semaphore pending here.
     *          do NOT stop/suspend this task */

}

#ifdef SYS_AOS_CLI_ENABLE

void _cli_init(int fd_console)
{
    ot_uartSetFd(fd_console);

#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_start(int fd_console);
    usb_cdc_start(fd_console);
#endif
}

#endif

#if defined(CFG_USB_CDC_ENABLE)

void usb_cdc_update_serial_number(uint32_t * pdeviceserial0, uint32_t * pdeviceserial1, uint32_t * pdeviceserial2) 
{
    uint8_t addr[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};

    bl_wireless_mac_addr_get(addr);
 
    *pdeviceserial0 = 0x0 + ((uint32_t *)addr)[0];
    *pdeviceserial1 = 0x1 + ((uint32_t *)addr)[1];
    *pdeviceserial2 = 0x2;

    (*pdeviceserial0) += (*pdeviceserial2);
}
#endif

int main(int argc, char *argv[])
{
    otRadio_opt_t opt;

#if defined(CFG_BLE_ENABLE)
    ble_stack_start();
#endif
    opt.byte = 0;

    opt.bf.isCoexEnable = true;
#if OPENTHREAD_RADIO
    opt.bf.isCoexEnable = false;
#endif

#if OPENTHREAD_FTD
    opt.bf.isFtd = true;
#endif

#if OPENTHREAD_CONFIG_MLE_LINK_METRICS_SUBJECT_ENABLE
    opt.bf.isLinkMetricEnable = true;
#endif
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    opt.bf.isCSLReceiverEnable = true;
#endif
#if OPENTHREAD_CONFIG_TIME_SYNC_ENABLE
    opt.bf.isTimeSyncEnable = true;
#endif

    otrStart(opt);
#ifdef CONFIG_OTDEMO
    app_task();
#endif

    return 0;
}
