#ifndef __BL_AUDIO_PDM_H__
#define __BL_AUDIO_PDM_H__

#include "bl702l_auadc.h"
#include "bl702l_glb.h"
#include "bl702l_dma.h"

typedef void (*bl_audio_pdm_callback_t)(int buf_idx);

// pdm_clk_pin: 0, 2, 8
// pdm_in_pin: 1, 3, 7
// pcm_frame_size: 1 - 4095
typedef struct {
    uint8_t pdm_clk_pin;
    uint8_t pdm_in_pin;
    uint16_t pcm_frame_size;
    int16_t *pcm_frame_buf[2];
    bl_audio_pdm_callback_t pcm_frame_event;
}bl_audio_pdm_cfg_t;

int bl_audio_pdm_init(bl_audio_pdm_cfg_t *cfg);
int bl_audio_pdm_start(void);
int bl_audio_pdm_stop(void);

#endif
