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
#ifndef __BL_AUDIO_AUADC_H__
#define __BL_AUDIO_AUADC_H__

#include "bl702l_auadc.h"
#include "bl702l_glb.h"
#include "bl702l_dma.h"

typedef void (*bl_audio_auadc_callback_t)(int buf_idx);

// auadc_pos_ch/auadc_neg_ch: 0 - 7, corresponding to gpio {2, 3, 7, 8, 10, 11, 14, 15}
// pcm_frame_size: 1 - 4095
typedef struct {
    uint8_t auadc_pos_ch;
    uint8_t auadc_neg_ch;
    uint16_t pcm_frame_size;
    int16_t *pcm_frame_buf[2];
    bl_audio_auadc_callback_t pcm_frame_event;
}bl_audio_auadc_cfg_t;

int bl_audio_auadc_init(bl_audio_auadc_cfg_t *cfg);
int bl_audio_auadc_start(void);
int bl_audio_auadc_stop(void);

#endif
