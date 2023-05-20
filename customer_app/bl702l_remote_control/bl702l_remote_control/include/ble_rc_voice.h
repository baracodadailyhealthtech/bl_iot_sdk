#ifndef __VOICE_H__
#define __VOICE_H__

#include "bl_audio.h"
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

#define VOICE_INTF                 1  // 1: PDM; 2: AMIC

#define PDM_CLK_PIN                8
#define PDM_IN_PIN                 3

#define AMIC_POS_CH                1  // i.e. GPIO3
#define AMIC_NEG_CH                3  // i.e. GPIO8

#define VOICE_DATA_PATTER_SIZE     6
#if defined (CONFIG_ATVV_SERVER_ENABLE)
#define ENCODED_VOICE_FRAME_SIZE   ATVV_AUDIO_FRAME_SIZE - VOICE_DATA_PATTER_SIZE
#define ORIG_VOICE_FRAME_SIZE      2 * ENCODED_VOICE_FRAME_SIZE
#else
#define ORIG_VOICE_FRAME_SIZE      256//320  //320 int16 = 640bytes
#define ENCODED_VOICE_FRAME_SIZE   128//160 //160bytes
#endif
#define NOTIFY_VOICE_DATA_SIZE     VOICE_DATA_PATTER_SIZE + ENCODED_VOICE_FRAME_SIZE 
#define ENCODED_BUFFER_CNT         15

int ble_rc_voice_start(void);
int ble_rc_voice_stop(void);
void ble_rc_voice_cfg(void);
#endif
