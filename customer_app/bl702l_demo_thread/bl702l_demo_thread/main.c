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

    uint8_t testNetworkkey[] = THREAD_NETWORK_KEY;
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
