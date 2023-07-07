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

#include <rom_api.h>
#include <rom_hal_ext.h>

#include <utils_list.h>
#include <openthread/thread.h>
#include <include/openthread_port.h>
#include <ot_utils_ext.h>

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
