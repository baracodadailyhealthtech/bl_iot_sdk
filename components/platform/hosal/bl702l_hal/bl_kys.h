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
#ifndef __BL_KYS_H__
#define __BL_KYS_H__

#include "bl702l_kys.h"
#include "bl702l_glb.h"

#define KEY_EVENT_NONE             0  // the key state is unchanged
#define KEY_EVENT_RELEASE          1  // the key state is changed from pressed state to released state
#define KEY_EVENT_PRESS            2  // the key state is changed from released state to pressed state

typedef struct {
    uint8_t ghost_det;
    uint8_t fifo_full;
    uint8_t key_num;
    uint8_t key_code[8];
    uint8_t row_idx[8];
    uint8_t col_idx[8];
}kys_result_t;

void bl_kys_init(uint8_t row_num, uint8_t col_num, uint8_t row_pins[], uint8_t col_pins[]);  // row_num: 1 - 8; col_num: 1 - 8
void bl_kys_trigger_poll(kys_result_t *result);
void bl_kys_trigger_interrupt(void);
void bl_kys_abort(void);
int bl_kys_is_key_event_detected(void);
int bl_kys_get_key_event_by_coordinates(uint8_t row_idx, uint8_t col_idx);
int bl_kys_get_key_event_by_key_code(uint8_t key_code);
void bl_kys_interrupt_callback(const kys_result_t *result);

#endif
