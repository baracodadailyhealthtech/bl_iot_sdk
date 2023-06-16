
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
