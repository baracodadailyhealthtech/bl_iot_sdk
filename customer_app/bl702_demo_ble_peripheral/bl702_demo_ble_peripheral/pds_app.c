#if defined(CFG_PDS_ENABLE)
#include <FreeRTOS.h>
#include <task.h>
#include <bl_irq.h>
#include <bl_rtc.h>
#include <bl_timer.h>
#include <bl_flash.h>
#include <hosal_uart.h>
#include "hal_pds.h"
#include "ble_lib_api.h"

#define  DEBUG_PRINT  1

#if (!DEBUG_PRINT)
#define printf(...)                (void)0
#endif
bool pds_start;

void pdsapp_init(void)
{
    hal_pds_init();
}

void pdsapp_enable_pds(bool enable)
{
    pds_start = enable;
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

}

static void pdsapp_restore(void)
{
   #if (DEBUG_PRINT)
    HOSAL_UART_DEV_DECL(uart_stdio, 0, 14, 15, 2000000);
    hosal_uart_init(&uart_stdio);
   #endif

    ble_controller_sleep_restore();
}

#if ( configUSE_TICKLESS_IDLE != 0 )
#define TIME_5MS_IN_32768CYCLE  (164) // (5000/(1000000/32768))
//Allocate retention memory
void *bl_alloc_retmem(size_t xWantedSize )
{
    return pvPortMalloc(xWantedSize);    
}

void vApplicationSleep(TickType_t xExpectedIdleTime_ms)
{
    int32_t bleSleepDuration_32768cycles = 0;
    int32_t expectedIdleTime_32768cycles = 0;
    eSleepModeStatus eSleepStatus;
    bool freertos_max_idle = false;
#if (LE_PDS_FLASH)
    uint8_t ioMode;
    uint8_t contRead; 
#endif

    if (pds_start == 0)
        return;

    if(xExpectedIdleTime_ms + xTaskGetTickCount() == portMAX_DELAY){
        freertos_max_idle = true;
    }else{   
        xExpectedIdleTime_ms -= 1;
        expectedIdleTime_32768cycles = 32768 * xExpectedIdleTime_ms / 1000;
    }

    if((!freertos_max_idle)&&(expectedIdleTime_32768cycles < TIME_5MS_IN_32768CYCLE)){
        return;
    }
        
    eSleepStatus = eTaskConfirmSleepModeStatus();
    if(eSleepStatus == eAbortSleep || ble_controller_sleep_is_ongoing())
    {
        return;
    }

    pdsapp_before_sleep_callback();

    bleSleepDuration_32768cycles = ble_controller_sleep(0);

    if(bleSleepDuration_32768cycles < TIME_5MS_IN_32768CYCLE)
    {
        pdsapp_sleep_aborted_callback();
        return;
    }
    else
    { 
        printf("Sleep_cycles=%ld\r\n", bleSleepDuration_32768cycles);
        uint8_t reduceSleepTime;
        SPI_Flash_Cfg_Type *flashCfg = bl_flash_get_flashCfg();
        uint8_t ioMode =  flashCfg->ioMode & 0xF;
        uint8_t contRead = flashCfg->cReadSupport;
        uint8_t cpuClk = GLB_Get_Root_CLK_Sel();
        if(ioMode == 4 && contRead == 1 && cpuClk == GLB_ROOT_CLK_XTAL)
        {
           reduceSleepTime = 100;
        }
        else if(ioMode == 1 && contRead == 0 && cpuClk == GLB_ROOT_CLK_XTAL)
        {
           #if (DEBUG_PRINT)
           reduceSleepTime = 130;
           #else
           //reduceSleepTime = 35;
           reduceSleepTime = 75;  // there is 1ms delay after xtal power on after pds wakeup
           #endif
        }
        else
        {
           reduceSleepTime = 130;
        }
        if(eSleepStatus == eStandardSleep && ((!freertos_max_idle) && (expectedIdleTime_32768cycles < bleSleepDuration_32768cycles)))
        {
           hal_pds_enter_with_time_compensation(31, expectedIdleTime_32768cycles - reduceSleepTime);
        }
        else
        {
           hal_pds_enter_with_time_compensation(31, bleSleepDuration_32768cycles - reduceSleepTime);
        }
        pdsapp_restore();

        pdsapp_after_sleep_callback();
        
    }
}
#endif
#endif//#if defined(CFG_PDS_ENABLE)
