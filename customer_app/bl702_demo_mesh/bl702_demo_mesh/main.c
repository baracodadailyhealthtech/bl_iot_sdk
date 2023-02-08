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

#include <bl_sys.h>
#include <bl_chip.h>
#include <bl_wireless.h>
#include <bl_irq.h>
#include <bl_sec.h>
#include <bl_rtc.h>
#include <bl_uart.h>
#include <bl_gpio.h>
#include <bl_flash.h>
#include <bl_timer.h>
#include <hal_boot2.h>
#include <hal_board.h>
#include <hosal_uart.h>
#include <hal_gpio.h>
#include <hal_hwtimer.h>
#include <hal_pds.h>
#include <hal_tcal.h>

#include <easyflash.h>
#include <libfdt.h>
#include <utils_log.h>
#include <blog.h>

#ifdef EASYFLASH_ENABLE
#include <easyflash.h>
#endif

#if defined(CFG_BLE_ENABLE)
#include "bluetooth.h"
#include "ble_cli_cmds.h"
#include <hci_driver.h>
#include "ble_lib_api.h"
#include "log.h"
#endif
#if defined(CONFIG_BT_OAD_SERVER)
#include "oad_main.h"
#include "oad_service.h"
#endif

HOSAL_UART_DEV_DECL(uart_stdio, 0, 14, 15, 2000000);

extern uint8_t _heap_start;
extern uint8_t _heap_size; // @suppress("Type cannot be resolved")
extern uint8_t _heap2_start;
extern uint8_t _heap2_size; // @suppress("Type cannot be resolved")
static HeapRegion_t xHeapRegions[] =
{
    { &_heap_start,  (unsigned int) &_heap_size}, //set on runtime
    { &_heap2_start, (unsigned int) &_heap2_size },            
    { NULL, 0 }, /* Terminates the array. */
    { NULL, 0 } /* Terminates the array. */
};

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName )
{
    puts("Stack Overflow checked\r\n");
	if(pcTaskName){
		printf("Stack name %s\r\n", pcTaskName);
	}
    while (1) {
        /*empty here*/
    }
}

void vApplicationMallocFailedHook(void)
{
    printf("Memory Allocate Failed. Current left size is %d bytes\r\n"
        ,xPortGetFreeHeapSize()
    );
    while (1) {
        /*empty here*/
    }
}

void vApplicationIdleHook(void)
{
    __asm volatile("wfi");
}

void bl_pds_restore(void)
{
    bl_uart_init(0, 14, 15, 255, 255, 2 * 1000 * 1000);
    bl_uart_int_enable(0);
#ifndef CFG_USB_CDC_ENABLE
	//power off Dll
    GLB_Power_Off_DLL();
#endif
#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_restore(void);
    usb_cdc_restore();
#endif
}

#if ( configUSE_TICKLESS_IDLE != 0 )
void vApplicationSleep( TickType_t xExpectedIdleTime )
{
    
}
#endif

#if ( configUSE_TICK_HOOK != 0 )
void vApplicationTickHook( void )
{
#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_monitor(void);
    usb_cdc_monitor();
#endif
}
#endif

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    //static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
    static StackType_t uxIdleTaskStack[256];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    //*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE; 
    *pulIdleTaskStackSize = 256;//size 256 words is For ble pds mode, otherwise stack overflow of idle task will happen.
}

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    /* If the buffers to be provided to the Timer task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void user_vAssertCalled(void) __attribute__ ((weak, alias ("vAssertCalled")));
void vAssertCalled(void)
{
    taskDISABLE_INTERRUPTS();

    abort();
}


static void _cli_init()
{
    /*Put CLI which needs to be init here*/
#if defined(CFG_EFLASH_LOADER_ENABLE)
    extern int helper_eflash_loader_cli_init(void);
    helper_eflash_loader_cli_init();
#endif

#if defined(CFG_RFPHY_CLI_ENABLE)
    extern int helper_rfphy_cli_init(void);
    helper_rfphy_cli_init();
#endif
}

static int get_dts_addr(const char *name, uint32_t *start, uint32_t *off)
{
    uint32_t addr = hal_board_get_factory_addr();
    const void *fdt = (const void *)addr;
    uint32_t offset;

    if (!name || !start || !off) {
        return -1;
    }

    offset = fdt_subnode_offset(fdt, 0, name);
    if (offset <= 0) {
       log_error("%s NULL.\r\n", name);
       return -1;
    }

    *start = (uint32_t)fdt;
    *off = offset;

    return 0;
}

#if defined(CFG_BLE_ENABLE)
extern int mesh_app_init(void);

#if defined(CONFIG_BT_OAD_SERVER)
bool app_check_oad(u32_t cur_file_ver, u32_t new_file_ver)
{
    //App layer decides whether to do oad according to file version
    /*if(new_file_ver > cur_file_ver)
        return true;
    else
        return false;*/
    return true;
}
#endif

void bt_enable_cb(int err)
{
    if (!err) {     
#ifdef CONFIG_BT_STACK_CLI 
        ble_cli_register();
#endif /* CONFIG_BT_STACK_CLI */
        #if defined(CONFIG_BT_OAD_SERVER)
        oad_service_enable(app_check_oad);
        #endif
		mesh_app_init();  
    }
}
void ble_init(void)
{
	 // Initialize BLE controller
    ble_controller_init(configMAX_PRIORITIES - 1);
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
}
#endif

static void aos_loop_proc(void *pvParameters)
{
    int fd_console;
    uint32_t fdt = 0, offset = 0;

#ifdef EASYFLASH_ENABLE
    easyflash_init();
#endif

    vfs_init();
    vfs_device_init();

    /* uart */
    const char *uart_node[] = {
        "uart@4000A000",
        "uart@4000A100",
    };

    if (0 == get_dts_addr("uart", &fdt, &offset)) {
        vfs_uart_init(fdt, offset, uart_node, 2);
    }

    /* gpio */
    if (0 == get_dts_addr("gpio", &fdt, &offset)) {
        hal_gpio_init_from_dts(fdt, offset);
    }

    aos_loop_init();

    fd_console = aos_open("/dev/ttyS0", 0);
    if (fd_console >= 0) {
        printf("Init CLI with event Driven\r\n");
        aos_cli_init(0);
        aos_poll_read_fd(fd_console, aos_cli_event_cb_read_get(), (void*)0x12345678);
        _cli_init();
    }

#if defined(CFG_USB_CDC_ENABLE)
    extern void usb_cdc_start(int fd_console);
    usb_cdc_start(fd_console);
#endif

    hal_hwtimer_init();

#if defined(CFG_BLE_ENABLE)
    ble_init();
#endif

    aos_loop_run();

    puts("------------------------------------------\r\n");
    puts("+++++++++Critical Exit From Loop++++++++++\r\n");
    puts("******************************************\r\n");
    vTaskDelete(NULL);
}

static void _dump_boot_info(void)
{
    char chip_feature[40];
    const char *banner;

    puts("Booting BL702 Chip...\r\n");

    /*Display Banner*/
    if (0 == bl_chip_banner(&banner)) {
//        puts(banner);
    }
    puts("\r\n");
    /*Chip Feature list*/
    puts("\r\n");
    puts("------------------------------------------------------------\r\n");
    puts("RISC-V Core Feature:");
    bl_chip_info(chip_feature);
    puts(chip_feature);
    puts("\r\n");

    puts("Build Version: ");
    puts(BL_SDK_VER); // @suppress("Symbol is not resolved")
    puts("\r\n");

    puts("Std BSP Driver Version: ");
    puts(BL_SDK_STDDRV_VER); // @suppress("Symbol is not resolved")
    puts("\r\n");

    puts("Std BSP Common Version: ");
    puts(BL_SDK_STDCOM_VER); // @suppress("Symbol is not resolved")
    puts("\r\n");

    puts("RF Version: ");
    puts(BL_SDK_RF_VER); // @suppress("Symbol is not resolved")
    puts("\r\n");

#if defined(CFG_BLE_ENABLE)
    puts("BLE Controller LIB Version: ");
    puts(ble_controller_get_lib_ver());
    puts("\r\n");
#endif

    puts("Build Date: ");
    puts(__DATE__);
    puts("\r\n");
    puts("Build Time: ");
    puts(__TIME__);
    puts("\r\n");
    puts("------------------------------------------------------------\r\n");

}

static void system_init(void)
{
    blog_init();
    bl_irq_init();

	bl_sec_init();

    bl_rtc_init();
    hal_boot2_init();

    /* board config is set after system is init*/
    hal_board_cfg(0);
    hal_pds_init();
    hal_tcal_init();
}

static void system_thread_init()
{
    /*nothing here*/
}

void rf_reset_done_callback(void)
{
    hal_tcal_restart();
}

void setup_heap()
{
    bl_sys_em_config();

    // Invoked during system boot via start.S
    vPortDefineHeapRegions(xHeapRegions);
}

void bl702_main()
{
    static StackType_t aos_loop_proc_stack[1024];
    static StaticTask_t aos_loop_proc_task;

    bl_sys_early_init();
    
    /*Init UART In the first place*/
    hosal_uart_init(&uart_stdio);
    puts("Starting bl702 now....\r\n");

    bl_sys_init();

    _dump_boot_info();

    printf("Heap %u@%p, %u@%p\r\n",
            (unsigned int)&_heap_size, &_heap_start,
            (unsigned int)&_heap2_size, &_heap2_start
    );

    system_init();
    system_thread_init();

    puts("[OS] Starting aos_loop_proc task...\r\n");
    xTaskCreateStatic(aos_loop_proc, (char*)"event_loop", sizeof(aos_loop_proc_stack)/4, NULL, 15, aos_loop_proc_stack, &aos_loop_proc_task);

    puts("[OS] Starting OS Scheduler...\r\n");
    vTaskStartScheduler();
}
