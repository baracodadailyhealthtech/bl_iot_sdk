#include <stdint.h>
#include <string.h>

#include "time_statics.h"

uint64_t g_ts_record[TS_MAX][5] = {0};

#ifdef TICKLESS_DEBUG
#define DUMP_RECORD_ITEM(x) printf( "[TSR]: %s:%lld,%lld,%lld,%lld,%lld \r\n", #x, g_ts_record[x][0], g_ts_record[x][1], g_ts_record[x][2], g_ts_record[x][3], g_ts_record[x][4])
#else
#define DUMP_RECORD_ITEM(x)
#endif

void time_static_record_dump(void) {
  /* TODO: dump record info */
  DUMP_RECORD_ITEM(TS_WANT_TO_SLEEP);
  DUMP_RECORD_ITEM(TS_ALLOW_SLEEP_WIFI);
  DUMP_RECORD_ITEM(TS_ALLOW_SLEEP_WIFI_DONE);
  DUMP_RECORD_ITEM(TS_ALLOW_SLEEP_BL_LP);
  DUMP_RECORD_ITEM(TS_WAKEUP_APP_START);
  DUMP_RECORD_ITEM(TS_WAKEUP_APP_DONE);
  DUMP_RECORD_ITEM(TS_WAKEUP_EXIT_TICKLESS);
  DUMP_RECORD_ITEM(TS_WAKEUP_RF_DONE);
#ifdef TICKLESS_DEBUG
  printf("                                                      ");
#endif
  memset(g_ts_record, 0, sizeof(g_ts_record));
}
