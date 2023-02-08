#ifndef __VOICE_H__
#define __VOICE_H__

#include "opus.h"
#include "opus_private.h"
#include "bl_audio_pdm.h"
#include "bl_port.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>


typedef struct
{
	uint8_t *buffer;
	uint32_t size;
	uint32_t wptr;
	uint32_t rptr;
}buffer_t;

#define PDM_CLK_PIN                8
#define PDM_IN_PIN                 3

#define ORIG_VOICE_FRAME_SIZE      256//320  //320 int16 = 640bytes
#define ENCODED_VOICE_FRAME_SIZE   128//160 //160bytes
#define VOICE_DATA_PATTER_SIZE     6
#define NOTIFY_VOICE_DATA_SIZE     VOICE_DATA_PATTER_SIZE + ENCODED_VOICE_FRAME_SIZE 
#define ENCODED_BUFFER_CNT         9

void ble_rc_voice_start(void);
void ble_rc_voice_cfg(void);
#endif
