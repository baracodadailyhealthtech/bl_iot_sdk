#include "bl_audio.h"
#include "bl_irq.h"
#include "ble_rc_voice.h"
#include "ble_rc_hog.h"
#include "log.h"
#include "adpcm.h"
#include "bl_port.h"
#include "gatt.h"
#include "ble_rc_app.h"
#include "ble_atv_voice.h"
#include "bl_uart.h"

#define VOICE_TASK_DESTROY_ONCE_VOICE_STOP 1
#define BLE_RC_VOICE_RAW_DATA_PRINT 0

T_IMA_ADPCM_STATE encode_state;
static int16_t pcm_buf[2][ORIG_VOICE_FRAME_SIZE];
extern bool voice_start; //Voice key have been pressed.

static struct k_thread voice_encode_task;
struct k_fifo voice_orig_data_queue;
static struct k_thread voice_tx_task;
struct k_fifo voice_encoded_data_queue;

unsigned char cbits[ENCODED_BUFFER_CNT][NOTIFY_VOICE_DATA_SIZE] = {0};
uint8_t eIndex = 0;
uint16_t block_seq = 0;
extern struct bt_conn *rc_default_conn;

void ble_rc_voice_frame_handle(int index)
{
    uint8_t queue_cnt = 0;
    if(rc_default_conn == NULL)
        return;
    //printf("[%lu]f\r\n",bl_timer_now_us());
    queue_cnt = k_queue_get_cnt(&voice_orig_data_queue);
    if(queue_cnt < 2)
        k_fifo_put_from_isr(&voice_orig_data_queue, pcm_buf[index]);
    else
        printf("voice_orig_data_queue full\r\n");
}

int ble_rc_voice_init(void)
{
    u8_t ret = 0;

#if VOICE_INTF == 1
    bl_audio_pdm_cfg_t cfg;

    cfg.pdm_clk_pin = PDM_CLK_PIN;
    cfg.pdm_in_pin = PDM_IN_PIN;
    cfg.pcm_frame_size = ORIG_VOICE_FRAME_SIZE;
    cfg.pcm_frame_buf[0] = pcm_buf[0];
    cfg.pcm_frame_buf[1] = pcm_buf[1];
    cfg.pcm_frame_event = ble_rc_voice_frame_handle;

    printf("bl_audio_pdm_init\r\n");
    ret = bl_audio_pdm_init(&cfg);
#endif

#if VOICE_INTF == 2
    bl_audio_amic_cfg_t cfg;

    cfg.amic_pos_ch = AMIC_POS_CH;
    cfg.amic_neg_ch = AMIC_NEG_CH;
    cfg.pcm_frame_size = ORIG_VOICE_FRAME_SIZE;
    cfg.pcm_frame_buf[0] = pcm_buf[0];
    cfg.pcm_frame_buf[1] = pcm_buf[1];
    cfg.pcm_frame_event = ble_rc_voice_frame_handle;

    printf("bl_audio_amic_init\r\n");
    ret = bl_audio_amic_init(&cfg);
#endif

    return ret;
}

static void ble_rc_encode_voice_task(void *pvParameters)
{
    int nbBytes = 0;
    unsigned int key = 0;
    u8_t queue_cnt = 0;
    while(1)
    {
        int16_t * data = k_fifo_get(&voice_orig_data_queue, K_FOREVER);
        if(data)
        {
            #if (BLE_RC_VOICE_RAW_DATA_PRINT)
            //printf voice raw data
            printf("\r\nrawdata start\r\n");
            unsigned int key = irq_lock();
            UART_SendData(0, (uint8_t *)data, ORIG_VOICE_FRAME_SIZE*sizeof(int16_t));
            irq_unlock(key);
            printf("\r\nrawdata end\r\n");
            #endif
            memset(cbits[eIndex], 0, NOTIFY_VOICE_DATA_SIZE);
            block_seq++;
            cbits[eIndex][0] = block_seq >> 8;
            cbits[eIndex][1] = (u8_t)block_seq;
            #if defined (CONFIG_ATVV_SERVER_ENABLE)
            cbits[eIndex][2] = ble_atvv_get_codec_or_stream_id();
            #endif
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
                queue_cnt = k_queue_get_cnt(&voice_encoded_data_queue);
                if(queue_cnt < ENCODED_BUFFER_CNT)
                {
                    //printf("[%lu]cnt=%d\r\n",bl_timer_now_us(),queue_cnt);
                    k_fifo_put(&voice_encoded_data_queue, cbits[eIndex]);
                }
                else
                    printf("voice_encoded_data_queue full\r\n");

                eIndex++;
                if(eIndex >= ENCODED_BUFFER_CNT)
                eIndex = 0;
            }
        }
    }
}

static void ble_rc_tx_task(void *pvParameters)
{
    struct bt_gatt_attr *attr;
    while(1)
    {
        int16_t * data = k_fifo_get(&voice_encoded_data_queue, K_FOREVER);
        if(data)
        {
            #if defined (CONFIG_ATVV_SERVER_ENABLE)
            attr = ble_atvv_get_attr(ATVV_CHAR_AUDIO_ATTR_VAL_INDEX);
            #else
            attr = ble_rc_get_voice_attr();
            #endif
            #if defined (CONFIG_ATVV_SERVER_ENABLE)
            if(ble_atvv_check_if_voice_notify())
            #endif
            {
                if(bt_gatt_notify(rc_default_conn, attr, data, NOTIFY_VOICE_DATA_SIZE))
                    printf("Failed to send voice data\r\n");
            }
        }
    }
}

#if (VOICE_TASK_DESTROY_ONCE_VOICE_STOP)
void ble_rc_voice_task_create(void)
{
    k_fifo_init(&voice_orig_data_queue, 2);
    k_thread_create(&voice_encode_task, "encode_voice", 1536,(k_thread_entry_t)ble_rc_encode_voice_task,configMAX_PRIORITIES - 4);
    k_fifo_init(&voice_encoded_data_queue, ENCODED_BUFFER_CNT);
    k_thread_create(&voice_tx_task, "tx_voice", 1536,(k_thread_entry_t)ble_rc_tx_task, configMAX_PRIORITIES - 3);
}

void ble_rc_voice_task_destroy(void)
{
    k_queue_free(&voice_orig_data_queue._queue);
    k_thread_delete(&voice_encode_task);
    k_queue_free(&voice_encoded_data_queue._queue);
    k_thread_delete(&voice_tx_task);
}
#endif

int ble_rc_voice_start(void)
{
    #if defined(CFG_BLE_PDS)
    GLB_Set_System_CLK(GLB_DLL_XTAL_32M, GLB_SYS_CLK_DLL128M);
    HBN_Set_XCLK_CLK_Sel(HBN_XCLK_CLK_XTAL);
    arch_delay_ms(1);
    #endif
    encode_state.index = 0;
    encode_state.valprev = 0;
    block_seq=0;
    #if (VOICE_TASK_DESTROY_ONCE_VOICE_STOP)
    ble_rc_voice_task_create();
    #else
    while(k_queue_get_cnt(&voice_orig_data_queue) != 0)
    {
        k_queue_get(&voice_orig_data_queue, K_NO_WAIT);
    }
    #endif
    
    ble_rc_voice_init();
    
    printf("bl_audio_start\r\n");
    return bl_audio_start();
}

int ble_rc_voice_stop(void)
{
    int err = bl_audio_stop();
    #if defined(CFG_BLE_PDS)
    GLB_Set_System_CLK(GLB_DLL_XTAL_32M, GLB_SYS_CLK_XTAL);
    arch_delay_ms(1);
    #endif
    printf("%s\r\n", __func__);
    #if (VOICE_TASK_DESTROY_ONCE_VOICE_STOP)
    ble_rc_voice_task_destroy();
    #endif
    return err;
}

void ble_rc_voice_cfg(void)
{
    ble_rc_voice_init();
    #if !(VOICE_TASK_DESTROY_ONCE_VOICE_STOP)
    k_fifo_init(&voice_orig_data_queue, 2);
    k_thread_create(&voice_encode_task, "encode_voice", 1536,(k_thread_entry_t)ble_rc_encode_voice_task,configMAX_PRIORITIES - 4);
    k_fifo_init(&voice_encoded_data_queue, ENCODED_BUFFER_CNT);
    k_thread_create(&voice_tx_task, "tx_voice", 1536,(k_thread_entry_t)ble_rc_tx_task, configMAX_PRIORITIES - 3);
    #endif
}
