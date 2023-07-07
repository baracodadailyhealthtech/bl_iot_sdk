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
#ifndef  __MAIN_H
#define  __MAIN_H

#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/cli.h>
#include <openthread_port.h>
#include <include/openthread_br.h>

#define THREAD_CHANNEL      11
#define THREAD_PANID        0x1234
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
