#include "bl_audio_pdm.h"
#include "hosal_dma.h"


static int8_t audio_dma_ch = -1;
static uint16_t pcm_frame_size;
static int16_t *pcm_frame_buf[2];
static bl_audio_pdm_callback_t pcm_frame_event;
static int pcm_frame_idx;


static void audio_dma_callback(void *p_arg, uint32_t flag)
{
    int idx = pcm_frame_idx;
    
    //BL_WR_REG(DMA_BASE, DMA_INTTCCLEAR, 1U << audio_dma_ch);
    
    pcm_frame_idx = !pcm_frame_idx;
    DMA_Channel_Update_DstMemcfg(DMA0_ID, audio_dma_ch, (uint32_t)pcm_frame_buf[pcm_frame_idx], pcm_frame_size);
    DMA_Channel_Enable(DMA0_ID, audio_dma_ch);
    
    if(pcm_frame_event != NULL){
        pcm_frame_event(idx);
    }
}


static void audio_gpio_init(bl_audio_pdm_cfg_t *cfg)
{
    GLB_GPIO_Cfg_Type gpioCfg;
    
    gpioCfg.gpioFun = GPIO_FUN_PDM;
    gpioCfg.gpioMode = GPIO_MODE_AF;
    gpioCfg.pullType = GPIO_PULL_NONE;
    gpioCfg.drive = 1;
    gpioCfg.smtCtrl = 1;
    
    gpioCfg.gpioPin = cfg->pdm_clk_pin;
    GLB_GPIO_Init(&gpioCfg);
    
    gpioCfg.gpioPin = cfg->pdm_in_pin;
    GLB_GPIO_Init(&gpioCfg);
}

static void audio_adc_init(bl_audio_pdm_cfg_t *cfg)
{
    AUADC_Cfg_Type auadc_cfg = {
        AUADC_CLK_16K_HZ,    /* sample rate */
        ENABLE,
        DISABLE,
        DISABLE,
        DISABLE,
        AUADC_SOURCE_PDM,
        AUADC_PDM_LEFT,
    };
    
    AUADC_FifoCfg_Type auadc_fifo_cfg = {
        AUADC_RES_16_BITS,
        AUADC_FIFO_AILGN_MSB_AT_BIT15,
        0,
        AUADC_DRQ_EQUEL_TO_IRQ,
        ENABLE,
    };
    
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_AUDIO);
    GLB_Set_AUDIO_CLK(0, 1, 0, 4);
    
    AUADC_SetVolume(10);
    
    AUADC_Disable();
    AUADC_Enable();
    
    AUADC_Init(&auadc_cfg);
    AUADC_FifoInit(&auadc_fifo_cfg);
}

static void audio_dma_init(bl_audio_pdm_cfg_t *cfg)
{
    DMA_Channel_Cfg_Type dmaChCfg = {
        0x4000AD88,                /* Source address of DMA transfer */
        0,                         /* Destination address of DMA transfer */
        0,                         /* Transfer length, 0~4095, this is burst count */
        DMA_TRNS_P2M,              /* Transfer dir control. 0: Memory to Memory, 1: Memory to peripheral, 2: Peripheral to memory */
        audio_dma_ch,              /* Channel select 0-7 */
        DMA_TRNS_WIDTH_16BITS,     /* Transfer width. 0: 8  bits, 1: 16  bits, 2: 32  bits */
        DMA_TRNS_WIDTH_16BITS,     /* Transfer width. 0: 8  bits, 1: 16  bits, 2: 32  bits */
        DMA_BURST_SIZE_1,          /* Number of data items for burst transaction length. Each item width is as same as tansfer width. 0: 1 item, 1: 4 items, 2: 8 items, 3: 16 items */
        DMA_BURST_SIZE_1,          /* Number of data items for burst transaction length. Each item width is as same as tansfer width. 0: 1 item, 1: 4 items, 2: 8 items, 3: 16 items */
        DISABLE,
        DISABLE,
        0,
        DMA_PINC_DISABLE,          /* Source address increment. 0: No change, 1: Increment */
        DMA_MINC_ENABLE,           /* Destination address increment. 0: No change, 1: Increment */
        DMA_REQ_AUADC,             /* Source peripheral select */
        DMA_REQ_NONE,              /* Destination peripheral select */
    };
    
    DMA_Channel_Init(DMA0_ID, &dmaChCfg);
    hosal_dma_irq_callback_set(audio_dma_ch, audio_dma_callback, NULL);
}


int bl_audio_pdm_init(bl_audio_pdm_cfg_t *cfg)
{
    if(cfg == NULL){
        return -1;
    }
    
    if(cfg->pdm_clk_pin != 0 && cfg->pdm_clk_pin != 2 && cfg->pdm_clk_pin != 8){
        return -1;
    }
    
    if(cfg->pdm_in_pin != 1 && cfg->pdm_in_pin != 3 && cfg->pdm_in_pin != 7){
        return -1;
    }
    
    if(cfg->pcm_frame_size < 1 || cfg->pcm_frame_size > 4095){
        return -1;
    }
    
    if(cfg->pcm_frame_buf[0] == NULL){
        return -1;
    }
    
    if(cfg->pcm_frame_buf[1] == NULL){
        return -1;
    }
    
    if(audio_dma_ch < 0){
        hosal_dma_init();
        
        audio_dma_ch = hosal_dma_chan_request(0);
        if(audio_dma_ch < 0){
            return -1;
        }
    }
    
    audio_gpio_init(cfg);
    audio_adc_init(cfg);
    audio_dma_init(cfg);
    
    pcm_frame_size = cfg->pcm_frame_size;
    pcm_frame_buf[0] = cfg->pcm_frame_buf[0];
    pcm_frame_buf[1] = cfg->pcm_frame_buf[1];
    pcm_frame_event = cfg->pcm_frame_event;
    
    return 0;
}

int bl_audio_pdm_start(void)
{
    if(audio_dma_ch < 0){
        return -1;
    }
    
    pcm_frame_idx = 0;
    DMA_Channel_Update_DstMemcfg(DMA0_ID, audio_dma_ch, (uint32_t)pcm_frame_buf[pcm_frame_idx], pcm_frame_size);
    DMA_Channel_Enable(DMA0_ID, audio_dma_ch);
    
    AUADC_Start();
    
    return 0;
}

int bl_audio_pdm_stop(void)
{
    if(audio_dma_ch < 0){
        return -1;
    }
    
    DMA_Channel_Disable(DMA0_ID, audio_dma_ch);
    
    AUADC_Stop();
    AUADC_FifoClear();
    
    return 0;
}
