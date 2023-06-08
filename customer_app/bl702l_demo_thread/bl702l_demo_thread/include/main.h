#ifndef  __MAIN_H
#define  __MAIN_H

#include <openthread/thread.h>
#include <openthread/cli.h>
#if OPENTHREAD_CONFIG_CHILD_SUPERVISION_ENABLE
#include <child_supervision.h>
#endif
#if OPENTHREAD_CONFIG_MAC_CSL_RECEIVER_ENABLE
#include <config/mac.h>
#endif

#define THREAD_CHANNEL          11
#define THREAD_PANID            0xB702
#define THREAD_UDP_PORT         0xB702
#ifdef CFG_PDS_ENABLE
#define THREAD_CSL_CHANNEL      20
#define THREAD_CSL_PERIOD       (1000000 / 160)
#ifdef CFG_CSL_RX
#define THREAD_POLL_PERIOD      30000
#else
#define THREAD_POLL_PERIOD      1000
#endif
#endif

#define THREAD_EXTPANID     	{0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22}
#define THREAD_NETWORK_KEY  	{0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}

void uart_init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate);
int printf_ram(const char *format, ...);

uint32_t app_pds_callback(otPds_op_t pdsOp, uint32_t sleepTimeMs, int isWakeupFromRtc);

void app_task(void);

#endif // __DEMO_GPIO_H
