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
