#include <stdio.h>

#include <bl_rtc.h>
#include <rom_api.h>
#include <rom_hal_ext.h>
#include <config/mac.h>
#include <include/openthread_port.h>
#include <ot_utils_ext.h>
#include <main.h>

#ifdef SYS_AOS_CLI_ENABLE

void _cli_init(int fd_console)
{
    ot_uartSetFd(fd_console);
}

#endif

ATTR_PDS_SECTION
void interrupt_entry_app(uint32_t mcause)
{
    void *handler = NULL;

    mcause &= 0x7FFFFFF;
    handler = bl_irq_handler_list[mcause];

    if (handler) {
        ((void (*)(void))handler)();
    } else {
        ATTR_PDS_SECTION static const char dbg_str[] = "mcause = %ld\r\n";

        uart_init(14, 15, 2000000);

        printf_ram(dbg_str, mcause);
        while (1) {
            /*dead loop now*/
        }
    }
}


ATTR_PDS_SECTION
void  exception_entry_app(uint32_t mcause, uint32_t mepc, uint32_t mtval, uintptr_t *regs, uintptr_t *tasksp)
{
    ATTR_PDS_SECTION static const char dbg_str[] = "Exception Entry--->>>\r\n mcause %08lx, mepc %08lx, mtval %08lx\r\n" ;

    uart_init(14, 15, 2000000);

    printf_ram(dbg_str, mcause, mepc, mtval);

    while (1) {
        /*dead loop now*/
    }
}

#ifdef CFG_PDS_ENABLE

static void ot_stateChangeCallback(uint32_t flags, void * p_context) 
{
    char states[5][10] = {"disabled", "detached", "child", "router", "leader"};
    otInstance *instance = (otInstance *)p_context;
    uint8_t *p;

    if (flags & OT_CHANGED_THREAD_ROLE)
    {

        uint32_t role = otThreadGetDeviceRole(p_context);

        if (role) {
            printf("Current role       : %s\r\n", states[otThreadGetDeviceRole(p_context)]);

            p = (uint8_t *)(otLinkGetExtendedAddress(instance)->m8);
            printf("Extend Address     : %02x%02x-%02x%02x-%02x%02x-%02x%02x\r\n", 
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);

            p = (uint8_t *)(otThreadGetMeshLocalPrefix(instance)->m8);
            printf("Local Prefx        : %02x%02x:%02x%02x:%02x%02x:%02x%02x\r\n",
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);

            p = (uint8_t *)(otThreadGetLinkLocalIp6Address(instance)->mFields.m8);
            printf("IPv6 Address       : %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x\r\n",
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);

            printf("Rloc16             : %x\r\n", otThreadGetRloc16(instance));

            p = (uint8_t *)(otThreadGetRloc(instance)->mFields.m8);
            printf("Rloc               : %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x\r\n",
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
        }
    }
}

void otrInitUser(otInstance * instance)
{
    otLinkModeConfig mode;

    printf("Thread version     : %s\r\n", otGetVersionString());

    uint8_t testNetworkkey[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
    otThreadSetNetworkKey(instance, (const otNetworkKey *)testNetworkkey);
    otLinkSetChannel(instance, THREAD_CHANNEL);

    otLinkSetPanId(instance, THREAD_PANID);

    memset(&mode, 0, sizeof(mode));
    mode.mDeviceType         = 0;
    mode.mNetworkData        = 0;
    mode.mRxOnWhenIdle       = 0;
#ifdef CFG_CSL_RX
    otLinkCslSetChannel(instance, THREAD_CSL_CHANNEL);
    otLinkCslSetPeriod(instance, THREAD_CSL_PERIOD);
#endif
#ifdef THREAD_POLL_PERIOD
    otLinkSetPollPeriod(instance, THREAD_POLL_PERIOD);
#endif
    otThreadSetLinkMode(instance, mode);

    otIp6SetEnabled(instance, true);
    otThreadSetEnabled(instance, true);

    printf("Link Mode           %d, %d, %d\r\n", 
        otThreadGetLinkMode(instance).mRxOnWhenIdle, 
        otThreadGetLinkMode(instance).mDeviceType, 
        otThreadGetLinkMode(instance).mNetworkData);
    printf("Link Mode           %d, %d, %d\r\n", 
        mode.mRxOnWhenIdle, mode.mDeviceType, mode.mNetworkData);
    printf("Network name        : %s\r\n", otThreadGetNetworkName(instance));
    printf("PAN ID              : %x\r\n", otLinkGetPanId(instance));

    printf("channel             : %d\r\n", otLinkGetChannel(instance));

    otSetStateChangedCallback(instance, ot_stateChangeCallback, instance);
}

#else

void otrInitUser(otInstance * instance)
{
#ifdef CONFIG_NCP
    otAppNcpInit((otInstance * )instance);
#else
    otAppCliInit((otInstance * )instance);
#endif
}
#endif

void otrAppProcess(ot_system_event_t sevent) 
{
    /** for application code */
    /** Note,   NO heavy execution, no delay and semaphore pending here.
     *          do NOT stop/suspend this task */

}

int main(int argc, char *argv[])
{
    otRadio_opt_t opt;

    /** comment out for debug purpose */
   // uint32_t tag = otrEnterCrit();

   // interrupt_entry_ptr = interrupt_entry_app;
   // exception_entry_ptr = exception_entry_app;
   // otrExitCrit(tag);

#ifdef CFG_PDS_ENABLE
    bl_pds_init();

    otPds_init(app_pds_callback);

#if OPENTHREAD_CONFIG_CHILD_SUPERVISION_ENABLE
    otPds_supervisionListener_init(OPENTHREAD_CONFIG_CHILD_SUPERVISION_CHECK_TIMEOUT);
#endif

#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
    otPds_cslReceiver_init(OPENTHREAD_CONFIG_CSL_RECEIVE_TIME_AHEAD, OPENTHREAD_CONFIG_CSL_MIN_RECEIVE_ON);
#endif

#if OPENTHREAD_CONFIG_MAC_ADD_DELAY_ON_NO_ACK_ERROR_BEFORE_RETRY
    otPds_setDelayBeforeRetry(OPENTHREAD_CONFIG_MAC_RETX_DELAY_MIN_BACKOFF_EXPONENT, OPENTHREAD_CONFIG_MAC_RETX_DELAY_MAX_BACKOFF_EXPONENT);
#endif

#endif

    ot_utils_init();

    opt.byte = 0;

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

    return 0;
}
