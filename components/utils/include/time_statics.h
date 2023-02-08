#ifndef __TIME_STATICS__
#define _TIME_STATICS__

#include <stdint.h>
#include "bl_rtc.h"

#include "rv_hpm.h"

enum {
  TS_WANT_TO_SLEEP,
  TS_ALLOW_SLEEP_WIFI,
  TS_ALLOW_SLEEP_WIFI_DONE,
  TS_ALLOW_SLEEP_BL_LP,
  TS_WAKEUP_APP_START,
  TS_WAKEUP_APP_DONE,
  TS_WAKEUP_EXIT_TICKLESS,
  TS_WAKEUP_RF_DONE,
  TS_MAX,
};

extern uint64_t g_ts_record[TS_MAX][5];

#ifdef TICKLESS_DEBUG
#define TS_RECORD(t) if (unlikely(g_ts_record[t][0] == 0)) { \
  g_ts_record[t][0] = bl_rtc_get_counter(); \
  RV_HPM_L1_ICache_Miss_Stop_M(&g_ts_record[t][1], &g_ts_record[t][2]); \
  RV_HPM_Cycle_Get_M(&g_ts_record[t][3]); \
  RV_HPM_Instret_Get_M(&g_ts_record[t][4]); \
}
#else
#define TS_RECORD(x)
#endif

void time_static_record_dump(void);
#endif
