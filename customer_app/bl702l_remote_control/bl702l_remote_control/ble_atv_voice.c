#if defined (CONFIG_ATVV_SERVER_ENABLE)
/** @file
 *  @brief Google ATV Voice Service
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr.h>
#include "settings.h"
#include "byteorder.h"
#include "bluetooth.h"
#include "hci_host.h"
#include "conn.h"
#include "uuid.h"
#include "gatt.h"
#include "log.h"
#include "ble_rc_hog.h"
#include "ble_rc_app.h"
#include "ble_rc_voice.h"
#include "ble_atv_voice.h"

extern struct bt_conn *rc_default_conn;
extern bool voice_start;
ble_atvv_env_t atvv_env;
static void ble_atvv_audio_stop(u8_t reason);
static void ble_atvv_audio_transfer_timer_cb(void *timer);

static int ble_atvv_notify(struct bt_conn *conn, uint8_t attr_index, uint8_t *buf, uint16_t len)
{
    int err = 0;

     if(rc_default_conn == NULL)
    {
        printf("%s, conn is null\r\n", __func__);
        return -1;
    }

    if(attr_index == ATVV_CHAR_CTL_ATTR_VAL_INDEX && atvv_env.ctlNotifyEnabled == false)
    {
        printf("%s, atvv ctlNotify is not enabled,not send notify\r\n", __func__);
        return -1;
    }
        
    err = bt_gatt_notify(conn, ble_atvv_get_attr(attr_index), buf, len);
     
    if (!err)
    {
        printf("atvv send notify success.");
    } else
    {
        printf("atvv send notify : %d", err);
    }

    return err;
}

void ble_atvv_send_caps_resp(struct bt_conn *conn)
{
    u8_t cmd_buf[9];
    u8_t idx = 0;

    //cmd
    cmd_buf[idx++] = ATVV_CHAR_CTL_CMD_CAPS_RESP;
    //version
    sys_put_be16(atvv_env.version_used, &cmd_buf[idx]);
    idx += 2;
    //sup_codecs
    cmd_buf[idx++] = ATVV_CODECS_ADPCM_16K_16BIT_MASK;
    //assis_model
    cmd_buf[idx++] = atvv_env.assis_model_used;
    //audio_frame_size
    sys_put_be16(ATVV_AUDIO_FRAME_SIZE, &cmd_buf[idx]);
    idx += 2;
    //extra_conf
    cmd_buf[idx++] = 1;
    //rsvd
    cmd_buf[idx] = 0;
    printf("ble_atvv_send_caps_resp\r\n");
    ble_atvv_notify(rc_default_conn, ATVV_CHAR_CTL_ATTR_VAL_INDEX, cmd_buf, sizeof(cmd_buf));
}

void ble_atvv_send_start_search(void)
{
    u8_t cmd_buf[1];

    cmd_buf[0] = ATVV_CHAR_CTL_CMD_START_SEARCH;
    ble_atvv_notify(rc_default_conn, ATVV_CHAR_CTL_ATTR_VAL_INDEX, cmd_buf, sizeof(cmd_buf));
}

static void ble_atvv_audio_start_notify_cb(struct bt_conn *conn, void *user_data)
{
    if(!atvv_env.audioTransTimer.timer.hdl)
    {
        k_timer_init(&atvv_env.audioTransTimer, ble_atvv_audio_transfer_timer_cb, NULL);
        k_timer_start(&atvv_env.audioTransTimer, pdMS_TO_TICKS(ATVV_AUDIO_TRANSFER_TIMEOUT_IN_SEC * 1000));
    }    
    else
    {
        k_timer_reset(&atvv_env.audioTransTimer);   
    }
    
    ble_rc_voice_start();
}

static int ble_atvv_send_audio_start(u8_t reason)
{
    u8_t cmd_buf[4];
    struct bt_gatt_notify_params params;

    if(!atvv_env.audioNotifyEnabled)
    {
        printf("Audio notify is not enabled by TV\r\n");
        return -1;
    }
    printf("audio_start,reason=%d\r\n",reason);
    cmd_buf[0] = ATVV_CHAR_CTL_CMD_AUTO_START;
    cmd_buf[1] = reason;
    cmd_buf[2] = ATVV_CODECS_ADPCM_16K_16BIT_MASK;

    if(reason == ATVV_ASSIS_MODEL_ON_REQ)
        cmd_buf[3] = 0;
    else
        cmd_buf[3] = atvv_env.stream_id;

    memset(&params, 0, sizeof(params));
    
    params.attr = ble_atvv_get_attr(ATVV_CHAR_CTL_ATTR_VAL_INDEX);
    params.data = cmd_buf;
    params.len = sizeof(cmd_buf);
    params.func = ble_atvv_audio_start_notify_cb;
    
    return bt_gatt_notify_cb(rc_default_conn, &params);
}

static void ble_atvv_send_mic_open_err(ble_atvv_err_code err_code)
{
    u8_t cmd_buf[2];

    cmd_buf[0] = ATVV_CHAR_CTL_CMD_MIC_OPEN_ERROR;
    cmd_buf[1] = err_code;

    ble_atvv_notify(rc_default_conn, ATVV_CHAR_CTL_ATTR_VAL_INDEX, cmd_buf, sizeof(cmd_buf));
}

static void ble_atvv_send_audio_stop(ble_atvv_audio_stop_reason reason)
{
    u8_t cmd_buf[2];
    
    cmd_buf[0] = ATVV_CHAR_CTL_CMD_AUTO_STOP;
    cmd_buf[1] = reason;

    ble_atvv_notify(rc_default_conn, ATVV_CHAR_CTL_ATTR_VAL_INDEX, cmd_buf, sizeof(cmd_buf));
}

static void ble_atvv_audio_transfer_timer_cb(void *timer)
{
    ble_atvv_audio_stop(ATVV_AUDIO_STOP_BY_TRANS_TIMEOUT);
}

static void ble_atvv_audio_start(u8_t reason)
{
    int err = 0;
    
    atvv_env.state = ATVV_STATE_AUDIO_START;
    atvv_env.mic_open_reason = reason;

    err = ble_atvv_send_audio_start(reason);
}

static void ble_atvv_audio_stop(u8_t reason)
{
    atvv_env.state = ATVV_STATE_AUDIO_STOP;
    if(atvv_env.audioTransTimer.timer.hdl)
    {
        k_timer_stop(&atvv_env.audioTransTimer);
        k_timer_delete(&atvv_env.audioTransTimer);
        atvv_env.audioTransTimer.timer.hdl = NULL;
    }
    
    ble_atvv_send_audio_stop(reason);
}

static void ble_atvv_recv_cmd_handler(u8_t *buf)
{
    u8_t cmd = buf[0];

    switch(cmd)
    {
        case ATVV_CHAR_TX_CMD_MIC_OPEN:
        {
            printf("recvd mic open\r\n");
            if(atvv_env.state == ATVV_STATE_START_SEARCH || 
              (atvv_env.state < ATVV_STATE_MIC_CLOSE  && atvv_env.mic_open_reason == ATVV_ASSIS_MODEL_ON_REQ))
            {
                atvv_env.mic_mode = buf[1];
                if(atvv_env.state < ATVV_STATE_MIC_CLOSE  && atvv_env.mic_open_reason == ATVV_ASSIS_MODEL_ON_REQ)
                {
                    /*if microphone is open because of a previously sent MIC_OPEN command, then the
                    Remote should restart an audio stream and notify Android TV host with AUDIO_START
                    message*/
                    ble_atvv_audio_stop(ATVV_AUDIO_STOP_BY_UPCOMING_AUDIO_START);
                }
                ble_atvv_audio_start(ATVV_ASSIS_MODEL_ON_REQ);
            }
            else
            {
                printf("received mic_open in unexpected state (%d),ignore", atvv_env.state);
                if(atvv_env.state < ATVV_STATE_MIC_CLOSE && atvv_env.mic_open_reason != ATVV_ASSIS_MODEL_ON_REQ)
                    ble_atvv_send_mic_open_err(ATVV_ERR_PTT_HTT_INPROG);
            }
        }
        break;

        case ATVV_CHAR_TX_CMD_MIC_CLOSE:
        {
            u8_t stream_id = buf[1];
            printf("recvd mic close\r\n");
            if(stream_id == ATVV_STREAM_ID_ANY || stream_id == atvv_env.stream_id)
            {
                if(atvv_env.state < ATVV_STATE_MIC_CLOSE)
                {
                    atvv_env.state = ATVV_STATE_MIC_CLOSE;
                    if(ble_atvv_get_assist_mode() != ATVV_ASSIS_MODEL_HTT)
                    {
                        voice_start = false;
                        ble_rc_voice_stop();
                        ble_atvv_voice_stop();
                    }
                }
            }
        }
        break;

        case ATVV_CHAR_TX_CMD_MIC_EXTEND:
        {
            u8_t stream_id = buf[1];
            printf("recvd mic extend\r\n");
            if(atvv_env.audioTransTimer.timer.hdl && (stream_id == ATVV_STREAM_ID_ANY || stream_id == atvv_env.stream_id))
            {
                k_timer_reset(&atvv_env.audioTransTimer);
            }
        }
        break;
    }
}

static ssize_t ble_atvv_char_tx_recv(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, u16_t len, u16_t offset,
                                 u8_t flags)
{
    u8_t atvv_cmd = ((u8_t *)buf)[0];
    printf("recvd atvv_cmd=%d\r\n",atvv_cmd);
    if(atvv_cmd == ATVV_CHAR_TX_CMD_GET_CAPS)
    {
        ble_atvv_get_caps_t caps_req;
        buf += sizeof(atvv_cmd);
        caps_req.version = sys_get_be16(buf);
        buf += sizeof(caps_req.version);
        caps_req.legacy = sys_get_be16(buf);
        buf += sizeof(caps_req.legacy);
        caps_req.sup_assis_models = *(u8_t *)buf;
        printf("caps_req.version=0x%x,legacy=%d,model=%d\r\n",caps_req.version, caps_req.legacy, caps_req.sup_assis_models);
        if(caps_req.version == ATVV_VERSION_1P0 && atvv_env.version_used == ATVV_VERSION_1P0)
            atvv_env.version_used = ATVV_VERSION_1P0;
        else
            atvv_env.version_used = ATVV_VERSION_0P4;

        if(atvv_env.version_used == ATVV_VERSION_0P4)
            atvv_env.assis_model_used = ATVV_ASSIS_MODEL_ON_REQ;
        else if(caps_req.sup_assis_models == ATVV_ASSIS_MODEL_ON_REQ_ONLY_SUP)
            atvv_env.assis_model_used = ATVV_ASSIS_MODEL_ON_REQ;
        else if(caps_req.sup_assis_models == ATVV_ASSIS_MODEL_ON_REQ_AND_PTT_SUP)
            atvv_env.assis_model_used = ATVV_ASSIS_MODEL_PTT;
        else if(caps_req.sup_assis_models == ATVV_ASSIS_MODEL_ON_REQ_AND_PTT_AND_HTT_SUP)
            atvv_env.assis_model_used = ATVV_ASSIS_MODEL_HTT;

        printf("assis_model_used=%d\r\n",atvv_env.assis_model_used);
        ble_atvv_send_caps_resp(conn);
        ble_rc_connection_update(ATVV_CONN_INTVAL_IN_NORMAL_MODE, ATVV_CONN_INTVAL_IN_NORMAL_MODE, ATVV_CONN_SLAVE_LATENCY, ATVV_CONN_TIMEOUT);
    }
    else
    {
        ble_atvv_recv_cmd_handler((u8_t *)buf);    
    }
    
    return len;
}

static void ble_atvv_audio_ccc_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    int err = 0;
    u8_t atvv_ccc = 0;
    if(value == BT_GATT_CCC_NOTIFY)
    {
        printf("enable atvv_audio notify\r\n");
        atvv_env.audioNotifyEnabled = true;
    }
    else
    {
        printf("disable atvv_audio notify\r\n");
        atvv_env.audioNotifyEnabled = false;
        if(atvv_env.state == ATVV_STATE_AUDIO_START)
        {
            ble_atvv_send_audio_stop(ATVV_AUDIO_STOP_BY_NOTIFY_DISABLED);
        }
    }

    atvv_ccc = (atvv_env.audioNotifyEnabled << ATVV_CCC_AUDIO_BIT_OFFSET) | (atvv_env.ctlNotifyEnabled << ATVV_CCC_CTL_BIT_OFFSET);
    err = bt_settings_set_bin(NV_ATVV_CCC, (const u8_t *)&atvv_ccc, sizeof(atvv_ccc));
    if(err)
        printf("Fails to store atv ccc value\r\n");
}

static void ble_atvv_ctl_ccc_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    int err = 0;
    u8_t atvv_ccc = 0;
    
    if(value == BT_GATT_CCC_NOTIFY)
        atvv_env.ctlNotifyEnabled = true;
    else
        atvv_env.ctlNotifyEnabled = false;

    atvv_ccc = (atvv_env.audioNotifyEnabled << ATVV_CCC_AUDIO_BIT_OFFSET) | (atvv_env.ctlNotifyEnabled << ATVV_CCC_CTL_BIT_OFFSET);
    err = bt_settings_set_bin(NV_ATVV_CCC, (const u8_t *)&atvv_ccc, sizeof(atvv_ccc));
    if(err)
        printf("Fails to store atv ccc value\r\n");
}
  
static struct bt_gatt_attr atvv_attrs[]= 
{
    /*primary server uuid*/
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ATVV_SERVICE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_ATVV_CHAR_TX, 
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL,
                           ble_atvv_char_tx_recv,
                           NULL),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_ATVV_CHAR_AUDIO,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, 
                           NULL, 
                           NULL),
                           
    BT_GATT_CCC(ble_atvv_audio_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
       
    BT_GATT_CHARACTERISTIC(BT_UUID_ATVV_CHAR_CTL, 
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, 
                           NULL, 
                           NULL),
                           
    BT_GATT_CCC(ble_atvv_ctl_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
};

static struct bt_gatt_service atvv_svc = BT_GATT_SERVICE(atvv_attrs);

struct bt_gatt_attr *ble_atvv_get_attr(u8_t index)
{
   return &atvv_attrs[index];
}

void ble_atvv_voice_start_handler_0p4(void)
{
    if(atvv_env.state == ATVV_STATE_VOICE_START_PENDING)
        ble_atvv_send_start_search();        
}

void ble_atvv_voice_start_handler_1p0(void)
{
    u8_t reason = ATVV_ASSIS_MODEL_PTT;

    atvv_env.stream_id++;
    if(atvv_env.stream_id > ATVV_STREAM_ID_MAX)
        atvv_env.stream_id = ATVV_STREAM_ID_MIN;

    if(atvv_env.assis_model_used == ATVV_ASSIS_MODEL_PTT)
        reason = ATVV_ASSIS_MODEL_PTT;
    else
        reason = ATVV_ASSIS_MODEL_HTT;
    
    ble_atvv_audio_start(reason);
}

void ble_atvv_voice_start_cont(void)
{
    printf("%s,ver=0x%x\r\n", __func__, atvv_env.version_used);    
    if(atvv_env.version_used == ATVV_VERSION_1P0)
    {
        ble_atvv_voice_start_handler_1p0();
    }
    else
    {
        ble_atvv_voice_start_handler_0p4();
    }
}

bool ble_atvv_if_voice_start_pending(void)
{
    if(atvv_env.state == ATVV_STATE_VOICE_START_PENDING)
        return true;
    else
        return false;
}

void ble_atvv_voice_start(void)
{
    int err = 0;
    
    if(atvv_env.state != ATVV_STATE_AUDIO_STOP)
    {
        printf("%s,voice is inprogress,return\r\n", __func__);
        return;
    }

    if(atvv_env.connUpdateTimer.timer.hdl)
    {
        printf("reset connUpdateTimer\r\n");
        k_timer_reset(&atvv_env.connUpdateTimer);
    }

    atvv_env.state = ATVV_STATE_VOICE_START_PENDING;
    err = ble_rc_connection_update(ATVV_CONN_INTVAL_IN_VOICE_MODE, ATVV_CONN_INTVAL_IN_VOICE_MODE, ATVV_CONN_SLAVE_LATENCY, ATVV_CONN_TIMEOUT);
    if(err)
    {
        ble_atvv_voice_start_cont();
    }            
}

static void ble_atvv_conn_update_timer_cb(void *timer)
{
    printf("%s hdl=%p\r\n", __func__,atvv_env.connUpdateTimer.timer.hdl);
    k_timer_delete(&atvv_env.connUpdateTimer);
    atvv_env.connUpdateTimer.timer.hdl = NULL;
    ble_rc_connection_update(ATVV_CONN_INTVAL_IN_NORMAL_MODE, ATVV_CONN_INTVAL_IN_NORMAL_MODE, ATVV_CONN_SLAVE_LATENCY, ATVV_CONN_TIMEOUT);
}

void ble_atvv_voice_stop(void)
{
    printf("%s,model=%d\r\n", __func__, atvv_env.assis_model_used);
    if(atvv_env.assis_model_used == ATVV_ASSIS_MODEL_HTT)
        ble_atvv_audio_stop(ATVV_AUDIO_STOP_BY_REL_BUTTON_IN_HTT);
    else
        ble_atvv_audio_stop(ATVV_AUDIO_STOP_BY_MIC_CLOSE);

    if(atvv_env.connUpdateTimer.timer.hdl == NULL)
    {
        k_timer_init(&atvv_env.connUpdateTimer, ble_atvv_conn_update_timer_cb, NULL);
        k_timer_start(&atvv_env.connUpdateTimer, pdMS_TO_TICKS(ATVV_CONN_UPDATE_TO_NORMAL_ITVL_TIMEOUT_IN_SEC * 1000));
    }
}

u8_t ble_atvv_get_assist_mode(void)
{
    return atvv_env.assis_model_used;
}

bool ble_atvv_check_if_voice_stop(void)
{
    if(atvv_env.state == ATVV_STATE_AUDIO_STOP)
        return true;
    else
        return false;
}

bool ble_atvv_check_if_voice_notify(void)
{
    if(atvv_env.audioNotifyEnabled && atvv_env.state != ATVV_STATE_AUDIO_STOP)
    {
        return true;
    }
    else
    {
        if(!atvv_env.audioNotifyEnabled)
            printf("atvv audio notify is disabled\r\n");
        else
            printf("atvv is in audio_stop mode\r\n");
        return false;
    }
}

u8_t ble_atvv_get_codec_or_stream_id(void)
{
    if(atvv_env.version_used == ATVV_VERSION_1P0)
        return atvv_env.codec_udsed;
    else
        return atvv_env.stream_id;  
}

static void ble_atvv_cfg(void)
{
    u8_t atvv_ccc = 0;
    
    atvv_env.version_used = ATVV_VERSION_1P0;
    atvv_env.assis_model_used = ATVV_ASSIS_MODEL_ON_REQ;
    atvv_env.state = ATVV_STATE_AUDIO_STOP;
    atvv_env.stream_id = 0;
    atvv_env.codec_udsed = ATVV_CODECS_ADPCM_16K_16BIT_MASK;
    bt_settings_get_bin(NV_ATVV_CCC, (u8_t*)&atvv_ccc, sizeof(atvv_ccc), NULL);
    atvv_env.audioNotifyEnabled = atvv_ccc & ATVV_CCC_AUDIO_BIT_MASK;
    atvv_env.ctlNotifyEnabled = atvv_ccc & ATVV_CCC_CTL_BIT_MASK;
}

void ble_atvv_init(void)
{
    bt_gatt_service_register(&atvv_svc);
    ble_atvv_cfg();
}
#endif//#if defined (CONFIG_ATVV_SERVER)
