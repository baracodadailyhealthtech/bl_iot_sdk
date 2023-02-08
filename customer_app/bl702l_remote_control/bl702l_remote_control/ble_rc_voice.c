#include "bl_audio_pdm.h"
#include "ble_rc_voice.h"
#include "ble_rc_hog.h"
#include "log.h"
#include "adpcm.h"
#include "bl_port.h"
#include "gatt.h"

T_IMA_ADPCM_STATE encode_state;
static int16_t pcm_buf[2][ORIG_VOICE_FRAME_SIZE];
extern bool voice_start; //Voice key have been pressed.

static struct k_thread voice_encode_task;
struct k_fifo voice_orig_data_queue;
unsigned char cbits[ENCODED_BUFFER_CNT][NOTIFY_VOICE_DATA_SIZE] = {0};
uint8_t eIndex = 0;
uint16_t block_seq = 0;
extern struct bt_conn *rc_default_conn;

void ble_rc_voice_frame_handle(int index)
{
    uint8_t queue_cnt = 0;
    if(rc_default_conn == NULL)
        return;
    
    queue_cnt = k_queue_get_cnt(&voice_orig_data_queue);
    if(queue_cnt < 2)
        k_fifo_put_from_isr(&voice_orig_data_queue, pcm_buf[index]);
}

int ble_rc_voice_pdm_gpio_init(void)
{
    bl_audio_pdm_cfg_t cfg;

    cfg.pdm_clk_pin = PDM_CLK_PIN;
    cfg.pdm_in_pin = PDM_IN_PIN;
    cfg.pcm_frame_size = ORIG_VOICE_FRAME_SIZE;
    cfg.pcm_frame_buf[0] = pcm_buf[0];
    cfg.pcm_frame_buf[1] = pcm_buf[1];
    cfg.pcm_frame_event = ble_rc_voice_frame_handle;
    printf("bl_audio_pdm_init\r\n");
    bl_audio_pdm_init(&cfg);
    return 0;
}

static void ble_rc_encode_voice_task(void *pvParameters)
{
    int nbBytes = 0;
    unsigned int key = 0;
    printf("%s\r\n", __func__);
    while(1)
    {
        int16_t * data = k_fifo_get(&voice_orig_data_queue, K_FOREVER);
        if(data)
        {
            memset(cbits[eIndex], 0, NOTIFY_VOICE_DATA_SIZE);
            block_seq++;
            cbits[eIndex][0] = block_seq >> 8;
            cbits[eIndex][1] = (u8_t)block_seq;
            cbits[eIndex][3] = encode_state.valprev >> 8;
            cbits[eIndex][4] = (u8_t)encode_state.valprev;
            cbits[eIndex][5] = encode_state.index;
            nbBytes = adpcm_coder(data, cbits[eIndex] + VOICE_DATA_PATTER_SIZE, ORIG_VOICE_FRAME_SIZE, &encode_state);
            if(nbBytes < 0)
            {
                printf("encode failed:\r\n");
            }
            else
            {
                struct bt_gatt_attr *attr = ble_rc_get_voice_attr();
                if(bt_gatt_notify(rc_default_conn, attr, cbits[eIndex], NOTIFY_VOICE_DATA_SIZE))
                    printf("Failed to send voice data\r\n");
            }
            eIndex++;
            if(eIndex >= ENCODED_BUFFER_CNT)
                eIndex = 0;
        }
    }
}

void ble_rc_voice_start(void)
{
    eIndex = 0;
    encode_state.index = 0;
    encode_state.valprev = 0;
    block_seq=0;
    while(k_queue_get_cnt(&voice_orig_data_queue) != 0)
    {
        k_queue_get(&voice_orig_data_queue, K_NO_WAIT);
    }
    for(int i=0;i<ENCODED_BUFFER_CNT;i++)
        memset(cbits[i],0,NOTIFY_VOICE_DATA_SIZE);
}

void ble_rc_voice_cfg(void)
{
    ble_rc_voice_pdm_gpio_init();
    k_fifo_init(&voice_orig_data_queue, 2);
    k_thread_create(&voice_encode_task, "encode_voice", 4096,(k_thread_entry_t)ble_rc_encode_voice_task,configMAX_PRIORITIES - 3);
}

