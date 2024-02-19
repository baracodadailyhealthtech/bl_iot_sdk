/*
 * Copyright (c) 2016-2024 Bouffalolab.
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
#ifndef __BL_OS_PORT_H__
#define __BL_OS_PORT_H__


#include <stdint.h>
#include <stdio.h>
#include <string.h>


void *bl_os_timer_create(const char *name, uint32_t period, void *handler, int repeat);
int bl_os_timer_start(void *xTimer);
int bl_os_timer_stop(void *xTimer);
int bl_os_timer_change_period(void *xTimer, uint32_t period);
int bl_os_timer_reset(void *xTimer);
int bl_os_timer_delete(void *xTimer);

void *bl_os_event_group_create(void);
uint32_t bl_os_event_group_wait_bits(void *xEventGroup, uint32_t bits, uint32_t timeout);
int bl_os_event_group_clear_bits(void *xEventGroup, uint32_t bits);
int bl_os_event_group_set_bits(void *xEventGroup, uint32_t bits);
int bl_os_event_group_delete(void *xEventGroup);


#endif
