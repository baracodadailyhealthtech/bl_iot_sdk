#ifndef __BLE_ATV_VOICE_H__
#define __BLE_ATV_VOICE_H__
#include "bl_port.h"

//AB5E0001-5A21-4F05-BC7D-AF01F617B664
#define BT_UUID_ATVV_SERVICE             BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xAB5E0001, 0x5A21, 0x4F05, 0xBC7D, 0xAF01F617B664))
//AB5E0002-5A21-4F05-BC7D-AF01F617B664
#define BT_UUID_ATVV_CHAR_TX             BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xAB5E0002, 0x5A21, 0x4F05, 0xBC7D, 0xAF01F617B664))
//AB5E0003-5A21-4F05-BC7D-AF01F617B664
#define BT_UUID_ATVV_CHAR_AUDIO          BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xAB5E0003, 0x5A21, 0x4F05, 0xBC7D, 0xAF01F617B664))
//AB5E0004-5A21-4F05-BC7D-AF01F617B664
#define BT_UUID_ATVV_CHAR_CTL            BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xAB5E0004, 0x5A21, 0x4F05, 0xBC7D, 0xAF01F617B664))

#define ATVV_CHAR_TX_ATTR_VAL_INDEX      2
#define ATVV_CHAR_AUDIO_ATTR_VAL_INDEX   4
#define ATVV_CHAR_CTL_ATTR_VAL_INDEX     7

#define ATVV_AUDIO_FRAME_SIZE  0x00A0
#define ATVV_VERSION_0P4    0x0004
#define ATVV_VERSION_1P0    0x0100

#define ATVV_STREAM_ID_MIN 0x01
#define ATVV_STREAM_ID_MAX 0x80
#define ATVV_STREAM_ID_ANY 0xFF

#define ATVV_CONN_INTVAL_IN_NORMAL_MODE  0x0020  //40ms
#define ATVV_CONN_INTVAL_IN_VOICE_MODE   0x0010  //20ms//0x000a
#define ATVV_CONN_TIMEOUT                0x01F4 //5s
#define ATVV_CONN_SLAVE_LATENCY          0x0000

#define ATVV_CCC_AUDIO_BIT_OFFSET 0x00
#define ATVV_CCC_CTL_BIT_OFFSET   0x01
#define ATVV_CCC_AUDIO_BIT_MASK   (1 << ATVV_CCC_AUDIO_BIT_OFFSET)
#define ATVV_CCC_CTL_BIT_MASK     (1 << ATVV_CCC_CTL_BIT_OFFSET)

#define ATVV_CODECS_ADPCM_8K_16BIT_MASK  0x01
#define ATVV_CODECS_ADPCM_16K_16BIT_MASK 0x02

#define ATVV_AUDIO_TRANSFER_TIMEOUT_IN_SEC    60 //60s
#define ATVV_CONN_UPDATE_TO_NORMAL_ITVL_TIMEOUT_IN_SEC   40 //40S

#define ATVV_ASSIS_MODEL_ON_REQ_ONLY_SUP             0x00
#define ATVV_ASSIS_MODEL_ON_REQ_AND_PTT_SUP          0x01
#define ATVV_ASSIS_MODEL_ON_REQ_AND_PTT_AND_HTT_SUP  0x03

#define ATVV_CHAR_TX_CMD_GET_CAPS                0x0A
#define ATVV_CHAR_TX_CMD_MIC_OPEN                0x0C
#define ATVV_CHAR_TX_CMD_MIC_CLOSE               0x0D
#define ATVV_CHAR_TX_CMD_MIC_EXTEND              0x0E
#define ATVV_CHAR_CTL_CMD_AUTO_STOP              0x00
#define ATVV_CHAR_CTL_CMD_AUTO_START             0x04
#define ATVV_CHAR_CTL_CMD_START_SEARCH           0x08
#define ATVV_CHAR_CTL_CMD_AUDIO_SYNC             0x0A
#define ATVV_CHAR_CTL_CMD_CAPS_RESP              0x0B
#define ATVV_CHAR_CTL_CMD_MIC_OPEN_ERROR         0x0C

typedef enum{
    ATVV_ASSIS_MODEL_ON_REQ = 0,
    ATVV_ASSIS_MODEL_PTT = 1,
    ATVV_ASSIS_MODEL_HTT = 3
}ble_atvv_assist_mode;

typedef enum{
    ATVV_STATE_VOICE_START_PENDING,
    ATVV_STATE_START_SEARCH,
    ATVV_STATE_MIC_OPEN,
    ATVV_STATE_AUDIO_START,
    ATVV_STATE_MIC_CLOSE,
    ATVV_STATE_AUDIO_STOP
}ble_atvv_state;

typedef enum{
    ATVV_MIC_MODE_PLAYBACK = 0,
    ATVV_MIC_MODE_CAPTURE
}ble_atvv_mic_mode;

typedef enum{
    ATVV_ERR_RSVD = 0x0F01,
    ATVV_ERR_REMOTE_NOT_ACTIVE,
    ATVV_ERR_NOTIFY_NOT_ENABLED,
    ATVV_ERR_PTT_HTT_INPROG = 0x0F80,
    ATVV_ERR_INTERNAL = 0x0FFF    
}ble_atvv_err_code;

typedef enum{
    ATVV_AUDIO_STOP_BY_MIC_CLOSE = 0x00,
    ATVV_AUDIO_STOP_BY_REL_BUTTON_IN_HTT = 0x02,
    ATVV_AUDIO_STOP_BY_UPCOMING_AUDIO_START = 0x04,
    ATVV_AUDIO_STOP_BY_TRANS_TIMEOUT = 0x04,
    ATVV_AUDIO_STOP_BY_NOTIFY_DISABLED = 0x10,
    ATVV_AUDIO_STOP_BY_OTHRE_REASON = 0x80
}ble_atvv_audio_stop_reason;

typedef struct{
    u16_t version_used;
    u8_t assis_model_used;
    u8_t mic_open_reason;
    u8_t stream_id;
    u8_t codec_udsed;
    k_timer_t audioTransTimer;
    k_timer_t connUpdateTimer;
    bool audioNotifyEnabled;
    bool ctlNotifyEnabled;
    ble_atvv_state state;
    ble_atvv_mic_mode mic_mode;
}ble_atvv_env_t;

typedef struct{
    u16_t version;
    u16_t legacy;
    u8_t sup_assis_models;
}ble_atvv_get_caps_t;

typedef struct{
    u16_t version;
    u8_t sup_codecs;
    u8_t assis_model;
    u16_t audio_frame_size;
    u8_t extra_conf;
    u8_t rsvd;
}ble_atvv_caps_resp_t;

typedef struct{
    u8_t reason;
    u8_t codec_used;
    u8_t stream_id;
}ble_atvv_audio_start_t;

void ble_atvv_init(void);
struct bt_gatt_attr *ble_atvv_get_attr(u8_t index);
u8_t ble_atvv_get_assist_mode(void);
bool ble_atvv_check_if_voice_notify(void);
bool ble_atvv_check_if_voice_stop(void);
bool ble_atvv_if_voice_start_pending(void);
u8_t ble_atvv_get_codec_or_stream_id(void);
void ble_atvv_voice_start(void);
void ble_atvv_voice_start_cont(void);
void ble_atvv_voice_stop(void);
#endif //__BLE_ATV_VOICE_H__