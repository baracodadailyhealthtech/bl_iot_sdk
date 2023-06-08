#ifndef  __MAIN_H
#define  __MAIN_H

#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/cli.h>
#include <openthread_port.h>
#include <include/openthread_br.h>

#define THREAD_CHANNEL      11
#define THREAD_PANID        0xB702
#define THREAD_EXTPANID     {0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22}
#define THREAD_NETWORK_KEY  {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}

#define CODE_CLI_START_THREAD  0x04

// #define WIFI_LWIP_RESET_PIN 11

void app_cli_cdc_monitor(void);
void app_cli_task(void);

void app_taskStart(void);

void wifi_lwip_hw_reset(void);
void wifi_lwip_init(void);
void cmd_connect(char *buf, int len, int argc, char **argv);
void cmd_disconnect(char *buf, int len, int argc, char **argv);
void cmd_scan(char *buf, int len, int argc, char **argv);
void cmd_get_info(char *buf, int len, int argc, char **argv);

void blsync_ble_start (void);
void blsync_ble_stop (void);

void eth_lwip_init(void);

void main_task_resume(void);

#endif // __DEMO_GPIO_H
