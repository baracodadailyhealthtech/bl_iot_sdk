/*
** adpcm.h - include file for adpcm coder.
**
** Version 1.0, 7-Jul-92.
*/
 
#ifndef ADPCM_H
#define ADPCM_H
 
#ifdef __cplusplus
extern "C" {
#endif
 
typedef struct {
    short	valprev;	/* Previous output value */
    char	index;		/* Index into stepsize table */
}T_IMA_ADPCM_STATE;

int adpcm_coder(short *indata, unsigned char *outdata, int len, T_IMA_ADPCM_STATE *state);
int adpcm_decoder(unsigned char *indata, short *outdata, int len, T_IMA_ADPCM_STATE *state);

//extern T_IMA_ADPCM_STATE ima_adpcm_global_state;

#ifdef __cplusplus
}  /* extern "C" */
#endif
 
#endif /* ADPCM_H*/
