/*
 * Copyright (c) 2016-2025 Bouffalolab.
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
#include <string.h>

#include <rom_hal_ext.h>

#include <openthread_port.h>
#include <ot_utils_ext.h>
#include <ot_rom_pds.h>
#include <ot_pds_ext.h>

#include "main.h"

ATTR_PDS_SECTION
uint32_t  otPds_prepareSleepOp(void) 
{
    if (otPds_isFullStackRunning()) {
        return otPds_prepareFullStackSleep();
    }
    else {
        return otPds_preparePdsStackSleep();
    }
}

ATTR_PDS_SECTION
uint32_t  app_pds_callback(otPds_op_t pdsOp, uint32_t sleepTimeMs, int isWakeupFromRtc) 
{
    switch (pdsOp) {

        case OT_PDS_SLEEP_OP_PRE_APP:
        {
            /** Add application sleep condition check code */
            if (otPds_isFullStackRunning()) {
                if (NULL == otrGetInstance()) {
                    return 0;
                }

                if (OT_DEVICE_ROLE_CHILD == otThreadGetDeviceRole(otrGetInstance())) {
                    return -1;
                }
                else {
                    return 0;
                }
                return -1;
            }
            else {
                return -1;
            }
            break;
        }
        case OT_PDS_SLEEP_OP_PRE_OT:
        {
            return otPds_prepareSleepOp();
        }
        break;
        case OT_PDS_SLEEP_OP_PRE_BLE:
        {
            return -1;
        }
        case OT_PDS_SLEEP_OP_APP_END:
        {
            /** Add application sleep code */
            return -1;
        }
        case OT_PDS_SLEEP_OP_OT_END:
        {
            otPds_sleepOp();
            return -1;
        }
        case OT_PDS_WAKEUP_OT_BLE:
        {
            break;
        }
        case OT_PDS_WAKEUP_OP_OT:
        {
            otPds_handleWakeup(isWakeupFromRtc);
            return -1;
        }
        case OT_PDS_WAKEUP_OP_APP:
        {
            /** Add application wakeup code */
        }

        break;

        default:
        return -1;
    }

    return -1;
}

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

void otrAppProcess(ot_system_event_t sevent) 
{
    /** for application code */
    /** Note,   NO heavy execution, no delay and semaphore pending here.
     *          do NOT stop/suspend this task */

}

int main(int argc, char *argv[])
{
    otRadio_opt_t opt;

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
