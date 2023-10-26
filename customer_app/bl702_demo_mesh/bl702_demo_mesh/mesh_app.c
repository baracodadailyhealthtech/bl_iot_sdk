/*
 * Copyright (c) 2016-2023 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdlib.h>
#include "conn.h"
#include "gatt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"
#include "mesh_cli_cmds.h"
#include "include/mesh.h"
#include "errno.h"

#include "mesh.h"
#include "access.h"
#include "net.h"
#include "transport.h"
#include "foundation.h"
#include "mesh_settings.h"
#include "adv.h"
#include "beacon.h"
#include "hci_core.h"
#include "log.h"
#include "bfl_ble_mesh_generic_model_api.h"
#include "bfl_ble_mesh_lighting_model_api.h"
#include "bfl_ble_mesh_local_data_operation_api.h"
#include "bfl_ble_mesh_networking_api.h"
#include "bfl_ble_mesh_time_scene_model_api.h"
#include "model_opcode.h"
#if defined(CONFIG_BT_SETTINGS)
#include "easyflash.h"
#include "settings.h"
#include "ef_def.h"
#define NV_LOCAL_ID_SCENE   "LOCAL_ID_SCENE"
#endif
#include <../../blestack/src/include/bluetooth/crypto.h>
#include "local_operation.h"
#include "hal_gpio.h"

#define CUR_FAULTS_MAX 4

u8_t dev_uuid[16] = {
    0xA8,0x01,0x71,0x5e,0x1c,0x00,0x00,0xe4,0x46,0x46,0x63,0xa7,0xf8,0x02,0x00,0x00
};
u8_t auth_value[16] = {
    0x78,0x8A,0xE3,0xEE,0x0F,0x2A,0x7E,0xFA,0xD3,0x67,0x35,0x81,0x41,0xFE,0x1B,0x06
};

static u8_t cur_faults[CUR_FAULTS_MAX];
static u8_t reg_faults[CUR_FAULTS_MAX * 2];

static struct {
    u16_t local;
    u16_t dst;
    u16_t net_idx;
    u16_t app_idx;
} net = {
    .local = BT_MESH_ADDR_UNASSIGNED,
    .dst = BT_MESH_ADDR_UNASSIGNED,
};

static struct bt_mesh_elem elements[];

static void prov_reset(void);
static void link_open(bt_mesh_prov_bearer_t bearer);
static void link_close(bt_mesh_prov_bearer_t bearer);
static int output_number(bt_mesh_output_action_t action, u32_t number);
static int output_string(const char *str);
static void prov_input_complete(void);
static void prov_complete(u16_t net_idx, u16_t addr);
static int input(bt_mesh_input_action_t act, u8_t size);
static void get_faults(u8_t *faults, u8_t faults_size, u8_t *dst, u8_t *count);
static int fault_get_cur(struct bt_mesh_model *model, u8_t *test_id,
                u16_t *company_id, u8_t *faults, u8_t *fault_count);
static int fault_get_reg(struct bt_mesh_model *model, u16_t cid,
                u8_t *test_id, u8_t *faults, u8_t *fault_count);
static int fault_clear(struct bt_mesh_model *model, uint16_t cid);
static int fault_test(struct bt_mesh_model *model,
                uint8_t test_id, uint16_t cid);
static void attn_on(struct bt_mesh_model *model);
static void attn_off(struct bt_mesh_model *model);

#ifdef CONFIG_BT_MESH_MOD_BIND_CB
static int prov_mod_bind_cb(struct bt_mesh_model *model, u16_t net_idx, u16_t mod_app_idx)
{
    int err = 0;
    if(model != bt_mesh_model_find(elements, BFL_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV)){
        BT_DBG("Model isn't primary model");
        return 0;
    }

    /* Check on off model has bound */
    BT_WARN("net_idx %x, mod_app_idx %x", net_idx, mod_app_idx);
    if(model->id == BT_MESH_MODEL_ID_CFG_CLI || model->id == BT_MESH_MODEL_ID_CFG_SRV){
        err = bt_mesh_local_model_bind_direct(net_idx, mod_app_idx);
    }

    return err;
}
#endif /* CONFIG_BT_MESH_MOD_BIND_CB */
#ifdef CONFIG_BT_MESH_APPKEY_ADD_CB
static int prov_app_key_add_cb(u16_t net_idx, u16_t mod_app_idx)
{
    BT_WARN("net_idx %x, mod_app_idx %x", net_idx, mod_app_idx);
    //bt_mesh_local_model_bind_direct(net_idx, mod_app_idx);
    return 0;
}
#endif /* CONFIG_BT_MESH_APPKEY_ADD_CB */


#ifdef CONFIG_BT_MESH_MOD_SUB_ADD_CB
static int prov_mod_sub_add_cb(struct bt_mesh_model *model, 
                u16_t elem_addr, u16_t group_addr)
{
    int i, j, err = 0;
    const struct bt_mesh_comp* dev_comp = bt_mesh_comp_get();
    u16_t addr = bt_mesh_primary_addr();

    if(model != bt_mesh_model_find(elements, BFL_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV)){
        BT_DBG("Model isn't primary model");
        return 0;
    }
    BT_WARN("elem_addr %x, group_addr %x", elem_addr, group_addr);

    for (i = 0; i < dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];
        struct bt_mesh_model *model;
    
        for (j = 0; j < elem->model_count; j++) {
            model = &elem->models[j];

            if(model->id == BT_MESH_MODEL_ID_CFG_CLI
                || model->id == BT_MESH_MODEL_ID_CFG_SRV){
                continue;
            }
            if(i == 0 && model->id == BFL_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV){
                continue;
            }
            err = bt_mesh_model_subscribe_group_addr(addr+i, 0xffff,
                model->id, group_addr);
            if(err != 0)
                BT_WARN("Err = %d", err);
        }

        for (j = 0; j < elem->vnd_model_count; j++) {
            model = &elem->vnd_models[j];

            err = bt_mesh_model_subscribe_group_addr(addr+i, model->vnd.company,
                    model->vnd.id, group_addr);
            if(err != 0)
                BT_WARN("Err = %d", err);
        }
    }
    return err;
}
#endif /* CONFIG_BT_MESH_APPKEY_ADD_CB */

static struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .oob_info = 0,
    .link_open = link_open,
    .link_close = link_close,
    .complete = prov_complete,
    .reset = prov_reset,
    .static_val = auth_value,
    .static_val_len = 16,
    .output_size = 6,
    .output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
    .output_number = output_number,
    .output_string = output_string,
    .input_size = 6,
    .input_actions = (BT_MESH_ENTER_NUMBER | BT_MESH_ENTER_STRING),
    .input = input,
    .input_complete = prov_input_complete,
#ifdef CONFIG_BT_MESH_MOD_BIND_CB
    .mod_bind_cb = prov_mod_bind_cb,
#endif /* CONFIG_BT_MESH_MOD_BIND_CB */
#ifdef CONFIG_BT_MESH_APPKEY_ADD_CB
    .app_key_add_cb = prov_app_key_add_cb,
#endif /* CONFIG_BT_MESH_APPKEY_ADD_CB */
#ifdef CONFIG_BT_MESH_MOD_SUB_ADD_CB
    .mod_sub_add_cb = prov_mod_sub_add_cb,
#endif /* CONFIG_BT_MESH_APPKEY_ADD_CB */
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .fault_get_cur = fault_get_cur,
    .fault_get_reg = fault_get_reg,
    .fault_clear = fault_clear,
    .fault_test = fault_test,
    .attn_on = attn_on,
    .attn_off = attn_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};


BT_MESH_HEALTH_PUB_DEFINE(health_pub, CUR_FAULTS_MAX);


static struct bt_mesh_cfg_srv cfg_srv = {
    .relay = BT_MESH_RELAY_ENABLED,
    .beacon = BT_MESH_BEACON_ENABLED,//BT_MESH_BEACON_DISABLED,
#if defined(CONFIG_BT_MESH_FRIEND)
    .frnd = BT_MESH_FRIEND_DISABLED,
#else
    .frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
    .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 6 transmissions with 20ms interval */
    .net_transmit = BT_MESH_TRANSMIT(5, 20),
    /* 3 transmissions with 20ms interval */
    .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

static struct bt_mesh_cfg_cli cfg_cli = {
};

BFL_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
static bfl_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
};

BFL_BLE_MESH_MODEL_PUB_DEFINE(level_pub, 2 + 3, ROLE_NODE);
bfl_ble_mesh_gen_level_srv_t level_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
};

BFL_BLE_MESH_MODEL_PUB_DEFINE(level1_pub, 2 + 3, ROLE_NODE);
bfl_ble_mesh_gen_level_srv_t level1_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
};

BFL_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 1, ROLE_NODE);
static bfl_ble_mesh_client_t onoff_client;

BFL_BLE_MESH_MODEL_PUB_DEFINE(lightness_pub, 2 + 3, ROLE_NODE);
static bfl_ble_mesh_light_lightness_state_t lightness_state;
static bfl_ble_mesh_light_lightness_srv_t lightness_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_RSP_BY_APP,
    .state = &lightness_state,
};

bfl_ble_mesh_light_lightness_setup_srv_t lightness_setup_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_RSP_BY_APP,
    .state = &lightness_state,
};

BFL_BLE_MESH_MODEL_PUB_DEFINE(ctl_pub, 2 + 3, ROLE_NODE);
static bfl_ble_mesh_light_ctl_state_t ctl_state;
static bfl_ble_mesh_light_ctl_srv_t ctl_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &ctl_state,
};

bfl_ble_mesh_light_ctl_setup_srv_t ctl_setup_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &ctl_state,
};

bfl_ble_mesh_light_ctl_temp_srv_t ctl_temp_server = {
    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &ctl_state,
};

BFL_BLE_MESH_MODEL_PUB_DEFINE(time_pub,10, ROLE_NODE);
BFL_BLE_MESH_MODEL_PUB_DEFINE(scene_pub,3, ROLE_NODE);
BFL_BLE_MESH_MODEL_PUB_DEFINE(scheduler_pub,10, ROLE_NODE);

bfl_ble_mesh_time_state_t  time_state;
bfl_ble_mesh_time_setup_srv_t time_setup_server={

    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &time_state,
};
bfl_ble_mesh_time_srv_t time_server={

    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &time_state,
};
bfl_ble_mesh_scene_register_t scene_register = {
    .scene_value = NET_BUF_SIMPLE(6),
};
bfl_ble_mesh_scenes_state_t  scenes_state={
    .scene_count = 1,
    .scenes = &scene_register,
};

bfl_ble_mesh_scene_setup_srv_t scene_setup_server={

    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &scenes_state,
};
bfl_ble_mesh_scene_srv_t scene_server={

    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &scenes_state,
};

bfl_ble_mesh_schedule_register_t schedule_register;

bfl_ble_mesh_scheduler_state_t scheduler_state = {
    .schedule_count = 1,
    .schedules = &schedule_register,
};

bfl_ble_mesh_scheduler_srv_t scheduler_server={

    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &scheduler_state,
};
bfl_ble_mesh_scheduler_setup_srv_t scheduler_setup_server={

    .rsp_ctrl.get_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = BFL_BLE_MESH_SERVER_AUTO_RSP,
    .state = &scheduler_state,
};

static struct bt_mesh_model sig_models[] = {
    BT_MESH_MODEL_CFG_SRV(&cfg_srv),
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BFL_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
    BFL_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),
    BFL_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub, &level_server),
    BFL_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SRV(&lightness_pub, &lightness_server),
    BFL_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SETUP_SRV(&lightness_pub, &lightness_setup_server),
    BFL_BLE_MESH_MODEL_LIGHT_CTL_SRV(&ctl_pub, &ctl_server),
    BFL_BLE_MESH_MODEL_LIGHT_CTL_SETUP_SRV(&ctl_pub, &ctl_setup_server),
    BFL_BLE_MESH_MODEL_TIME_SRV(&time_pub,&time_server),
    BFL_BLE_MESH_MODEL_TIME_SETUP_SRV(&time_setup_server),
    BFL_BLE_MESH_MODEL_SCENE_SRV(&scene_pub,&scene_server),
    BFL_BLE_MESH_MODEL_SCENE_SETUP_SRV(&scene_pub,&scene_setup_server),
    BFL_BLE_MESH_MODEL_SCHEDULER_SRV(&scheduler_pub,&scheduler_server),
    BFL_BLE_MESH_MODEL_SCHEDULER_SETUP_SRV(&scheduler_pub,&scheduler_setup_server),

};

static struct bt_mesh_model sig_models1[] = {
    BFL_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level1_pub, &level1_server),
    BFL_BLE_MESH_MODEL_LIGHT_CTL_TEMP_SRV(&ctl_pub, &ctl_temp_server),
};

struct vendor_data_t{
    uint8_t data[BT_MESH_TX_VND_SDU_MAX_SHORT];
};
static struct vendor_data_t vendor_data;
static struct vendor_data_t vendor_data_cli;
int vendor_srv_update(struct bt_mesh_model *mod)
{
    BT_WARN("company[%x] id[%x]", mod->vnd.company, mod->vnd.id);
    bt_mesh_model_msg_init(mod->pub->msg, BLE_MESH_MODEL_VND_OP_DATA_STATUS);
    net_buf_simple_add_mem(mod->pub->msg, (uint8_t*)mod->user_data, 8);
    bt_mesh_model_publish(mod);
    return true;
}

BT_MESH_MODEL_PUB_DEFINE(vendor_srv_pub, vendor_srv_update, BT_MESH_TX_VND_SDU_MAX_SHORT);

static void vendor_data_set(struct bt_mesh_model *model,
                              struct bt_mesh_msg_ctx *ctx,
                              struct net_buf_simple *buf)
{
    NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_TX_SDU_MAX);

    memcpy((uint8_t*)model->user_data, buf->data, buf->len);
    BT_WARN("data[%s]", bt_hex(buf->data, buf->len));

    if (buf == NULL) {
        BT_ERR("%s, Invalid model user_data", __func__);
        return;
    }

    bt_mesh_model_msg_init(&msg, BLE_MESH_MODEL_VND_OP_DATA_STATUS);
    net_buf_simple_add_mem(&msg, buf->data, buf->len);

    bt_mesh_model_send(model, ctx, &msg, NULL, NULL);
}

static void vendor_data_status(struct bt_mesh_model *model,
                struct bt_mesh_msg_ctx *ctx,
                struct net_buf_simple *buf)
{
    BT_WARN("Vendor status[%s]", bt_hex(buf->data, buf->len));
}

/* Mapping of message handlers for Generic Power OnOff Server (0x1006) */
const struct bt_mesh_model_op vendor_data_op[] = {
    { BLE_MESH_MODEL_VND_OP_DATA_SET, 1, vendor_data_set },
    { BLE_MESH_MODEL_VND_OP_DATA_SET_UNACK, 1, vendor_data_set },
    BT_MESH_MODEL_OP_END,
};

const struct bt_mesh_model_op vendor_data_op_cli[] = {
    { BLE_MESH_MODEL_VND_OP_DATA_STATUS, 1, vendor_data_status },
    BT_MESH_MODEL_OP_END,
};


static struct bt_mesh_model vendor_models[] = {
    BT_MESH_MODEL_VND(BL_COMP_ID, BT_MESH_VND_MODEL_ID_DATA_SRV,
        vendor_data_op, &vendor_srv_pub, &vendor_data),
    BT_MESH_MODEL_VND(BL_COMP_ID, BT_MESH_VND_MODEL_ID_DATA_CLI,
        vendor_data_op_cli, NULL, &vendor_data_cli),
};
static struct bt_mesh_model vendor_models1[0];

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, sig_models, vendor_models),
    BT_MESH_ELEM(1, sig_models1, vendor_models1),
};

static const struct bt_mesh_comp comp = {
    .cid = BL_COMP_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static const char *bearer2str(bt_mesh_prov_bearer_t bearer)
{
    switch (bearer) {
    case BT_MESH_PROV_ADV:
        return "PB-ADV";
    case BT_MESH_PROV_GATT:
        return "PB-GATT";
    case BT_MESH_PROV_GATT_ADV:
        return "PB-GATT&PB-ADV";
    default:
        return "unknown";
    }
}

static void link_open(bt_mesh_prov_bearer_t bearer)
{
    BT_WARN("Provisioning link opened on %s", bearer2str(bearer));
}

static void link_close(bt_mesh_prov_bearer_t bearer)
{
    BT_WARN("Provisioning link closed on %s", bearer2str(bearer));
}

static int output_number(bt_mesh_output_action_t action, u32_t number)
{
    BT_WARN("OOB Number: %lu", number);
    return 0;
}

static int output_string(const char *str)
{
    BT_WARN("OOB String: %s", str);
    return 0;
}

static void prov_input_complete(void)
{
    BT_WARN("Input complete");
}

static void prov_complete(u16_t net_idx, u16_t addr)
{
    BT_WARN("Local node provisioned, net_idx 0x%04x address 0x%04x", net_idx, addr);
    net.net_idx = net_idx,
    net.local = addr;
    net.dst = addr;
}

static void prov_reset(void)
{
    BT_WARN("The local node has been reset and needs reprovisioning");
    if (!bt_mesh_is_provisioned()) 
    {
		BT_WARN("blemesh not init\n");
		return;
	}
	bt_mesh_reset();
}

#if defined(CONFIG_BT_MESH_MODEL)
#if defined(CONFIG_BT_MESH_MODEL_GEN_CLI)
static void blemeshcli_gen_oo_cli(char *pcWriteBuffer,
                int xWriteBufferLen, int argc, char **argv)
{
    static u8_t tid;
    bfl_ble_mesh_generic_client_set_state_t gen_client_set;
    bfl_ble_mesh_generic_client_get_state_t gen_client_get;
    
    if(argc < 1){
        BT_WARN("Number of Parameters is not correct");
        return;
    }

    bfl_ble_mesh_client_common_param_t onoff_common = {
        .ctx.app_idx = 0,
        .ctx.net_idx = 0,
        .ctx.send_ttl = 3,
        .msg_timeout = 0,
        .msg_role = 0,
        .model = NULL,
        .opcode = BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
    };

    onoff_common.model = bt_mesh_model_find(elements, BFL_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI);
    if(onoff_common.model == NULL){
        BT_WARN("Can't find gen onoff cli");
        return;
    }

    get_uint16_from_string(&argv[1], &onoff_common.ctx.addr);

    switch(argc){
    case 2: /* Get message */
        onoff_common.opcode = BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_GET;
        bfl_ble_mesh_generic_client_get_state(&onoff_common, &gen_client_get);
        break;
    case 4:  /* Set ack or unack with optional */
        if(strcmp(argv[3], "ack") == 0){
            onoff_common.opcode = BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET;
        }
        else if(strcmp(argv[3], "unack") == 0){
            onoff_common.opcode = BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
        }
        else{
            BT_WARN("Set ack/unack parameter error");
        }
        __attribute__ ((fallthrough));
    case 3:  /* Set ack or unack with optional */
        gen_client_set.onoff_set.op_en = false;
        get_uint8_from_string(&argv[2], &gen_client_set.onoff_set.onoff);
        gen_client_set.onoff_set.tid = tid++;
        bfl_ble_mesh_generic_client_set_state(&onoff_common, &gen_client_set);
        break;
    case 6:  /* Set ack or unack with optional */
        if(strcmp(argv[5], "ack") == 0){
            onoff_common.opcode = BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET;
        }
        else if(strcmp(argv[5], "unack") == 0){
            onoff_common.opcode = BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
        }
        else{
            BT_WARN("Set ack/unack parameter error");
        }
        __attribute__ ((fallthrough));
    case 5:  /* Set ack or unack message*/
        gen_client_set.onoff_set.op_en = true;
        get_uint8_from_string(&argv[2], &gen_client_set.onoff_set.onoff);
        get_uint8_from_string(&argv[3], &gen_client_set.onoff_set.trans_time);
        get_uint8_from_string(&argv[4], &gen_client_set.onoff_set.delay);
        gen_client_set.onoff_set.tid = tid++;
        bfl_ble_mesh_generic_client_set_state(&onoff_common, &gen_client_set);
        break;
    default :
        BT_WARN("Parameter error");
        break;
    }
}
#endif

static void blemesh_vendor_data_send(uint16_t addr, uint8_t* data, int len)
{
    struct bt_mesh_msg_ctx ctx = {
        .send_ttl = 3,
        .app_idx = 0,
        .net_idx = 0,
        .addr = addr,
    };

    if(data == NULL || len == 0)
        return;

    BT_WARN("enter");
    BT_MESH_MODEL_BUF_DEFINE(msg, BLE_MESH_MODEL_VND_OP_DATA_SET_UNACK,
        BT_MESH_TX_VND_SDU_MAX_SHORT);
    bt_mesh_model_msg_init(&msg, BLE_MESH_MODEL_VND_OP_DATA_SET_UNACK);


    net_buf_simple_add_mem(&msg, data, len);
    struct bt_mesh_model* model_t;
    model_t = bt_mesh_model_find_vnd(elements, BL_COMP_ID, BT_MESH_VND_MODEL_ID_DATA_CLI);
    if(model_t == NULL){
        BT_ERR("Unable to found vendor model");
    }

    if (bt_mesh_model_send(model_t, &ctx, &msg, NULL, NULL)){
        BT_ERR("Unable to send vendor cli command");
    }
}

static void blemeshcli_vendor_cli(char *pcWriteBuffer,
                int xWriteBufferLen, int argc, char **argv)
{
    u16_t addr;

    if(argc != 3){
        BT_WARN("Number of Parameters is not correct");
        return;
    }

    get_uint16_from_string(&argv[1], &addr);

    blemesh_vendor_data_send(addr, (uint8_t*)argv[2], strlen(argv[2]));
}

#endif /* CONFIG_BT_MESH_MODEL */

static void blemeshcli_reset(char *pcWriteBuffer,
                int xWriteBufferLen, int argc, char **argv)
{
    bt_mesh_reset();
    BT_WARN("Local node reset complete");
}

static int input(bt_mesh_input_action_t act, u8_t size)
{
    switch (act) {
    case BT_MESH_ENTER_NUMBER:{
        BT_WARN("Enter a number (max %u digits) with: input-num <num>:", size);
    }break;
    case BT_MESH_ENTER_STRING:{
        BT_WARN("Enter a string (max %u chars) with: input-str <str>", size);
    }break;
    default:
        BT_WARN("Unknown input action %u (size %u) requested!", act, size);
        return -EINVAL;
    }
    return 0;
}

static void get_faults(u8_t *faults, u8_t faults_size, u8_t *dst, u8_t *count)
{
    u8_t i, limit = *count;

    for (i = 0U, *count = 0U; i < faults_size && *count < limit; i++) {
        if (faults[i]) {
            *dst++ = faults[i];
            (*count)++;
        }
    }
}

void show_faults(u8_t test_id, u16_t cid, u8_t *faults, size_t fault_count)
{
    if (!fault_count) {
        BT_WARN("Health Test ID 0x%02x Company ID 0x%04x: no faults",
            test_id, cid);
        return;
    }

    BT_WARN("Health Test ID 0x%02x Company ID 0x%04x Fault Count %zu:",
        test_id, cid, fault_count);

    BT_WARN("Fault %s", bt_hex(faults, fault_count));
}

static int fault_get_cur(struct bt_mesh_model *model, u8_t *test_id,
                u16_t *company_id, u8_t *faults, u8_t *fault_count)
{
    BT_WARN("Sending current faults");

    *test_id = 0x00;
    *company_id = BT_COMP_ID_LF;

    get_faults(cur_faults, sizeof(cur_faults), faults, fault_count);

    return 0;
}

static int fault_get_reg(struct bt_mesh_model *model, u16_t cid,
                u8_t *test_id, u8_t *faults, u8_t *fault_count)
{
    if (cid != BT_COMP_ID_LF) {
        BT_WARN("Faults requested for unknown Company ID 0x%04x", cid);
        return -EINVAL;
    }

    BT_WARN("Sending registered faults");

    *test_id = 0x00;

    get_faults(reg_faults, sizeof(reg_faults), faults, fault_count);

    return 0;
}

static int fault_clear(struct bt_mesh_model *model, uint16_t cid)
{
    if (cid != BT_COMP_ID_LF) {
        return -EINVAL;
    }

    (void)memset(reg_faults, 0, sizeof(reg_faults));

    return 0;
}

static int fault_test(struct bt_mesh_model *model, uint8_t test_id,
                uint16_t cid)
{
    if (cid != BT_COMP_ID_LF) {
        return -EINVAL;
    }

    if (test_id != 0x00) {
        return -EINVAL;
    }

    return 0;
}

static void attn_on(struct bt_mesh_model *model)
{
    BT_WARN("Attention timer on");
}

static void attn_off(struct bt_mesh_model *model)
{
    BT_WARN("Attention timer off");
}

static void model_gen_cb(uint8_t value)
{
    value?hal_gpio_led_on():hal_gpio_led_off();
}

static void example_ble_mesh_generic_server_cb(
                bfl_ble_mesh_generic_server_cb_event_t event,
                bfl_ble_mesh_generic_server_cb_param_t *param)
{
    switch(event){
    case BFL_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        if (param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            BT_WARN("Onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            model_gen_cb(param->value.state_change.onoff_set.onoff);
        }
        else if (param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
            param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
            BT_WARN("Level %d", param->value.state_change.level_set.level);
        }
        break;
    default:
        BT_WARN("Unknown Generic Server event 0x%02x, opcode 0x%04lx, src 0x%04x, dst 0x%04x",
            event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);
        break;
    }
}

static void ble_mesh_generic_onoff_client_model_cb(
                bfl_ble_mesh_generic_client_cb_event_t event,
                bfl_ble_mesh_generic_client_cb_param_t *param)
{
    uint32_t opcode = param->params->opcode;

    switch (event) {
    case BFL_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT: {
        switch (opcode) {
        case BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
            if (param->error_code == BFL_OK) {
                BT_WARN("GenOnOffClient:GetStatus,OK[%x]", param->status_cb.onoff_status.present_onoff);
            } else {
                BT_WARN("GenOnOffClient:GetStatus,Fail[%x]", param->error_code);
            }
            break;
        default:
            break;
        }
        break;
    }
    case BFL_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT: {
        switch (opcode) {
        case BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
            if (param->error_code == BFL_OK) {
                BT_WARN("GenOnOffClient:SetStatus,OK[%x]", param->status_cb.onoff_status.present_onoff);
            } else {
                BT_WARN("GenOnOffClient:SetStatus,Fail[%x]", param->error_code);
            }
            break;
        case BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
            if (param->error_code == BFL_OK) {
                BT_WARN("GenOnOffClient:SetUNACK,OK, opcode[%lx] raddr[%x]", 
                                opcode, param->params->ctx.addr);
            } else {
                BT_WARN("GenOnOffClient:SetUNACK,Fail[%x]", param->error_code);
            }
            break;
        default:
            break;
        }
        break;
    }
    case BFL_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT: {
        if (param->error_code == BFL_OK) {
            switch (opcode) {
            case BFL_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS:
                BT_WARN("Recv onoff status, raddr[%x]", 
                        param->params->ctx.addr);
            break;
            }
        } else {
            BT_WARN("GenOnOffClient:Publish,Fail[%x]", param->error_code);
        }
        break;
    }
    case BFL_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        BT_WARN("GenOnOffClient:TimeOut[%x]", param->error_code);
        break;
    case BFL_BLE_MESH_GENERIC_CLIENT_EVT_MAX:
        BT_WARN("GenONOFFClient:InvalidEvt[%x]", param->error_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_lighting_server_cb(
                bfl_ble_mesh_lighting_server_cb_event_t event,
                bfl_ble_mesh_lighting_server_cb_param_t *param)
{
    switch (event) {
    case BFL_BLE_MESH_LIGHTING_SERVER_STATE_CHANGE_EVT:
        if (param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET ||
            param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK) {
            BT_WARN("Light lightness [%x]",
                param->value.state_change.lightness_set.lightness);
        }
        else if (param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_LIGHT_CTL_SET ||
                 param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_LIGHT_CTL_SET_UNACK) {
            bfl_ble_mesh_state_change_light_ctl_set_t* set;
            set = &param->value.state_change.ctl_set;
            BT_WARN("Light ctl ln[%x]tp[%x]uv[%x]", 
                 set->lightness, set->temperature, set->delta_uv);
        }
        else if (param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET ||
                 param->ctx.recv_op == BFL_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET_UNACK) {
            bfl_ble_mesh_state_change_light_ctl_temperature_set_t* set;
            set = &param->value.state_change.ctl_temp_set;
            BT_WARN("Light ctl tp[%x] uv[%x]", 
                    set->temperature, set->delta_uv);
        }
        break;
    default:
        BT_WARN("Unknown Generic Server event 0x%02x, opcode 0x%04lx, src 0x%04x, dst 0x%04x",
            event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);
        break;
    }
}

static void example_ble_mesh_time_scene_server_cb(
        bfl_ble_mesh_time_scene_server_cb_event_t event,
        bfl_ble_mesh_time_scene_server_cb_param_t *param)
{
    switch(event)
    {
        case  BFL_BLE_MESH_TIME_SCENE_SERVER_STATE_CHANGE_EVT:
        {
            if ( param->ctx.recv_op ==BLE_MESH_MODEL_OP_TIME_SET)
            {
                BT_WARN("time set tai_seconds: [%x] [%x] [%x] [%x] [%x], subsecond[%x] uncertainty[%x] time_authority[%x] tai_utc_delta_curr[%x]", 
                param->value.state_change.time_set.tai_seconds[0],
                param->value.state_change.time_set.tai_seconds[1],
                param->value.state_change.time_set.tai_seconds[2],
                param->value.state_change.time_set.tai_seconds[3],
                param->value.state_change.time_set.tai_seconds[4],
                param->value.state_change.time_set.subsecond,
                param->value.state_change.time_set.uncertainty, 
                param->value.state_change.time_set.time_authority,
                param->value.state_change.time_set.tai_utc_delta_curr);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_TIME_ZONE_SET)
            { 
                BT_WARN("time_zone_offset_new [%x] tai_zone_change[%x] [%x] [%x] [%x] [%x]", 
                param->value.state_change.time_zone_set.time_zone_offset_new,
                param->value.state_change.time_zone_set.tai_zone_change[0],
                param->value.state_change.time_zone_set.tai_zone_change[1],
                param->value.state_change.time_zone_set.tai_zone_change[2],
                param->value.state_change.time_zone_set.tai_zone_change[3],
                param->value.state_change.time_zone_set.tai_zone_change[4]);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_TAI_UTC_DELTA_SET)
            { 
                BT_WARN("tai_utc_delta_new [%x] tai_delta_change[%x] [%x] [%x] [%x] [%x]", 
                param->value.state_change.tai_utc_delta_set.tai_utc_delta_new,
                param->value.state_change.tai_utc_delta_set.tai_delta_change[0],
                param->value.state_change.tai_utc_delta_set.tai_delta_change[1],
                param->value.state_change.tai_utc_delta_set.tai_delta_change[2],
                param->value.state_change.tai_utc_delta_set.tai_delta_change[3],
                param->value.state_change.tai_utc_delta_set.tai_delta_change[4]);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_TIME_ROLE_SET)
            { 
                BT_WARN("time_role_set [%x]", param->value.state_change.time_role_set.time_role);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_STORE||
                     param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_STORE_UNACK)
            { 
                BT_WARN("scene_store.scene_number [%x]", param->value.state_change.scene_store.scene_number);
                #if defined(CONFIG_BT_SETTINGS)
                bt_settings_set_bin(NV_LOCAL_ID_SCENE,(uint8_t*)&scene_register,sizeof(scene_register));
                #endif
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_DELETE||
                     param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_DELETE_UNACK)
            { 
                BT_WARN("scene_delete.scene_number[%x]", param->value.state_change.scene_delete.scene_number);
                #if defined(CONFIG_BT_SETTINGS)
                bt_settings_set_bin(NV_LOCAL_ID_SCENE,(uint8_t*)&scene_register,sizeof(scene_register));
                #endif
            }
            else if(param->ctx.recv_op == BLE_MESH_MODEL_OP_SCHEDULER_ACT_SET||
                    param->ctx.recv_op == BLE_MESH_MODEL_OP_SCHEDULER_ACT_SET_UNACK)
            { 
                BT_WARN("cheduler_act_set action [%x] day [%x] day_of_week [%x] hour [%x] index [%x] minute [%x] month [%x] scene_number [%x] second [%x] trans_time [%x] year [%x]",
                param->value.state_change.scheduler_act_set.action,
                param->value.state_change.scheduler_act_set.day,
                param->value.state_change.scheduler_act_set.day_of_week,
                param->value.state_change.scheduler_act_set.hour,
                param->value.state_change.scheduler_act_set.index,
                param->value.state_change.scheduler_act_set.minute,
                param->value.state_change.scheduler_act_set.month,
                param->value.state_change.scheduler_act_set.scene_number,
                param->value.state_change.scheduler_act_set.second,
                param->value.state_change.scheduler_act_set.trans_time,
                param->value.state_change.scheduler_act_set.year
                );
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_RECALL||
                     param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_RECALL_UNACK)
            { 
                BT_WARN("scene_recall.scene_number [%x]", param->value.state_change.scene_recall.scene_number);
            }
            break;
        }
        case  BFL_BLE_MESH_TIME_SCENE_SERVER_RECV_GET_MSG_EVT:
        {
            if ( param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCHEDULER_ACT_GET)
            {
                //#param->value.get.scheduler_act.index
            }
            break;
        }   
        case  BFL_BLE_MESH_TIME_SCENE_SERVER_RECV_SET_MSG_EVT:
        
            if ( param->ctx.recv_op ==BLE_MESH_MODEL_OP_TIME_SET)
            {
                BT_WARN("time set tai_seconds [%x] [%x] [%x] [%x] [%x]subsecond[%x] uncertainty[%x] time_authority[%x] tai_utc_delta[%x] time_zone_offset [%x]", 
                param->value.set.time.tai_seconds[0],
                param->value.set.time.tai_seconds[1],
                param->value.set.time.tai_seconds[2],
                param->value.set.time.tai_seconds[3],
                param->value.set.time.tai_seconds[4],
                param->value.set.time.subsecond,
                param->value.set.time.uncertainty, 
                param->value.set.time.time_authority,
                param->value.set.time.tai_utc_delta,
                param->value.set.time.time_zone_offset);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_TIME_ZONE_SET)
            {
                BT_WARN("time_zone_offset_new [%x] tai_zone_change[%x] [%x] [%x] [%x] [%x]", 
                param->value.set.time_zone.time_zone_offset_new,
                param->value.set.time_zone.tai_zone_change[0],
                param->value.set.time_zone.tai_zone_change[1],
                param->value.set.time_zone.tai_zone_change[2],
                param->value.set.time_zone.tai_zone_change[3],
                param->value.set.time_zone.tai_zone_change[4]);

            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_TAI_UTC_DELTA_SET)
            {
                BT_WARN("tai_utc_delta_new [%x] tai_utc_delta.padding [%x] tai_delta_change[%x] [%x] [%x] [%x] [%x]", 
                param->value.set.tai_utc_delta.tai_utc_delta_new,
                param->value.set.tai_utc_delta.padding,
                param->value.set.tai_utc_delta.tai_delta_change[0],
                param->value.set.tai_utc_delta.tai_delta_change[1],
                param->value.set.tai_utc_delta.tai_delta_change[2],
                param->value.set.tai_utc_delta.tai_delta_change[3],
                param->value.set.tai_utc_delta.tai_delta_change[4]);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_TIME_ROLE_SET)
            { 
                BT_WARN("time_role_set [%x]", param->value.set.time_role.time_role);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_STORE||
                     param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_STORE_UNACK)
            { 
                BT_WARN("scene_store.scene_number [%x]", param->value.set.scene_store.scene_number);
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_DELETE||
                     param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_DELETE_UNACK)
            { 
                BT_WARN("scene_delete.scene_number[%x]", param->value.set.scene_delete.scene_number);
            }
            else if(param->ctx.recv_op == BLE_MESH_MODEL_OP_SCHEDULER_ACT_SET||
                    param->ctx.recv_op == BLE_MESH_MODEL_OP_SCHEDULER_ACT_SET_UNACK)
            { 
                BT_WARN("cheduler_act_set action [%x] day [%x] day_of_week [%x] hour [%x] index [%x] minute [%x] month [%x] scene_number [%x] second [%x] trans_time [%x] year [%x]",
                param->value.set.scheduler_act.action,
                param->value.set.scheduler_act.day,
                param->value.set.scheduler_act.day_of_week,
                param->value.set.scheduler_act.hour,
                param->value.set.scheduler_act.index,
                param->value.set.scheduler_act.minute,
                param->value.set.scheduler_act.month,
                param->value.set.scheduler_act.scene_number,
                param->value.set.scheduler_act.second,
                param->value.set.scheduler_act.trans_time,
                param->value.set.scheduler_act.year
                );
            }
            else if (param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_RECALL||
                    param->ctx.recv_op ==BLE_MESH_MODEL_OP_SCENE_RECALL_UNACK)
            { 
                BT_WARN("scene_recall.scene_number [%x]", param->value.set.scene_recall.scene_number);
            }
         
            break;

        case  BFL_BLE_MESH_TIME_SCENE_SERVER_RECV_STATUS_MSG_EVT:
        {   
            if ( param->ctx.recv_op == BLE_MESH_MODEL_OP_TIME_STATUS)
            {
                BT_WARN("time set tai_seconds [%x] [%x] [%x] [%x] [%x]subsecond[%x] uncertainty[%x] time_authority[%x] tai_utc_delta[%x] time_zone_offset [%x]", 
                param->value.status.time_status.tai_seconds[0],
                param->value.status.time_status.tai_seconds[1],
                param->value.status.time_status.tai_seconds[2],
                param->value.status.time_status.tai_seconds[3],
                param->value.status.time_status.tai_seconds[4],
                param->value.status.time_status.subsecond,
                param->value.status.time_status.uncertainty, 
                param->value.status.time_status.time_authority,
                param->value.status.time_status.tai_utc_delta,
                param->value.status.time_status.time_zone_offset);
            }

            break;
        }
        default:
        {
            BT_WARN("Unknown Generic Server event 0x%02x, opcode 0x%04lx, src 0x%04x, dst 0x%04x",
            event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

            break;
        }
    }

}
static void unprov_stop_work_timeout_ck(struct k_work *work)
{
    BT_WARN("%d", bt_mesh_is_provisioned());
    if(!bt_mesh_is_provisioned()){
        bt_mesh_prov_disable(BT_MESH_PROV_GATT_ADV);
    }

    k_delayed_work_free((struct k_delayed_work*)work);
}

int mesh_app_init(void)
{
    static struct k_delayed_work unprov_stop_work;
    int err;
    bt_mesh_prov_bearer_t bearer = BT_MESH_PROV_GATT_ADV;
    #if defined(CONFIG_BT_SETTINGS)
    bt_settings_get_bin(NV_LOCAL_ID_SCENE, (uint8_t*)&scene_register, sizeof(scene_register), NULL);
    #endif
    bfl_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);
    bfl_ble_mesh_register_generic_client_callback(ble_mesh_generic_onoff_client_model_cb);

    bfl_ble_mesh_register_lighting_server_callback(example_ble_mesh_lighting_server_cb);
    bfl_ble_mesh_register_time_scene_server_callback(example_ble_mesh_time_scene_server_cb);
    /* For test */
    bt_addr_le_t adv_addr;
    bt_get_local_public_address(&adv_addr);
    u8_t uuid[16] = {0x07,0xaf,0x00,0x00,0x11,0x11,0x22,0x22,0x33,0x33,
                            adv_addr.a.val[5],
                            adv_addr.a.val[4],
                            adv_addr.a.val[3],
                            adv_addr.a.val[2],
                            adv_addr.a.val[1],
                            adv_addr.a.val[0]};
    memcpy(dev_uuid, uuid, 16);
    
    err = bt_mesh_init(&prov, &comp);
    if(err){
        BT_WARN("Failed to init mesh");
        return -1;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        mesh_set();
        mesh_commit();
    }

    if (bt_mesh_is_provisioned()) {
        BT_WARN("Mesh network restored from flash");
    } else {
        err = bt_mesh_prov_enable(bearer);
        if (err) {
            BT_WARN("Failed to enable %s (err %d)", bearer2str(bearer), err);
        } else {
            BT_WARN("%s enabled", bearer2str(bearer));

            k_delayed_work_init(&unprov_stop_work, unprov_stop_work_timeout_ck);
            k_delayed_work_submit(&unprov_stop_work, 10*60*1000);
        }
    }

    return 0;
}

#if defined(BL602) || defined(BL702)
const struct cli_command btMeshCmdSet[] STATIC_CLI_CMD_ATTRIBUTE = {
#else
const struct cli_command btMeshCmdSet[] = {
#endif
    {"blemesh_reset", "", blemeshcli_reset},
    {"blemesh_gen_oo_cli", "", blemeshcli_gen_oo_cli},
    {"blemesh_vendor_cli", "", blemeshcli_vendor_cli},
#if defined(BL70X)
    {NULL, NULL, "No handler / Invalid command", NULL}
#endif
};


