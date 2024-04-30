/*
 * Copyright (c) 2016-2024 Bouffalolab.
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
#if defined(CONFIG_BT_MESH_OTA_TARGET)

#include "hosal_ota.h"
#include "hal_sys.h"
#include "mesh_ota.h"

static struct bt_mesh_blob_io_flash blob_flash_stream;
static uint32_t target_fw_ver_curr = 0xDEADBEEF;
static uint32_t target_fw_ver_new;
static struct bt_mesh_dfu_img dfu_imgs[] = { {
	.fwid = &target_fw_ver_curr,
	.fwid_len = sizeof(target_fw_ver_curr),
} };

static int mesh_ota_target_metadata_check(struct bt_mesh_dfu_srv *srv,
			  const struct bt_mesh_dfu_img *img,
			  struct net_buf_simple *metadata_raw,
			  enum bt_mesh_dfu_effect *effect)
{
	//need to modify according to the real metadata
	memcpy(&target_fw_ver_new, net_buf_simple_pull_mem(metadata_raw, sizeof(target_fw_ver_new)),
	       sizeof(target_fw_ver_new));
	return 0;
}

static int mesh_ota_target_dfu_start(struct bt_mesh_dfu_srv *srv,
		     const struct bt_mesh_dfu_img *img,
		     struct net_buf_simple *metadata,
		     const struct bt_mesh_blob_io **io)
{
	*io = &blob_flash_stream.io;
	return 0;
}

static void mesh_ota_target_dfu_transfer_end(struct bt_mesh_dfu_srv *srv, const struct bt_mesh_dfu_img *img,
				    bool success)
{
	//To do verification of new image
	if(!hosal_ota_finish(1,0)){
		bt_mesh_dfu_srv_verified(srv);
	}else{
		bt_mesh_dfu_srv_rejected(srv);
	}
}

static int mesh_ota_target_dfu_recover(struct bt_mesh_dfu_srv *srv,
		       const struct bt_mesh_dfu_img *img,
		       const struct bt_mesh_blob_io **io)
{
	*io = &blob_flash_stream.io;
	return 0;
}

static int mesh_ota_target_dfu_apply(struct bt_mesh_dfu_srv *srv, const struct bt_mesh_dfu_img *img)
{
	bt_mesh_dfu_srv_applied(srv);
	target_fw_ver_curr = target_fw_ver_new;
	hal_reboot();
	return 0;
}

static const struct bt_mesh_dfu_srv_cb dfu_handlers = {
	.check = mesh_ota_target_metadata_check,
	.start = mesh_ota_target_dfu_start,
	.end = mesh_ota_target_dfu_transfer_end,
	.apply = mesh_ota_target_dfu_apply,
	.recover = mesh_ota_target_dfu_recover,
};

struct bt_mesh_dfu_srv dfu_srv = BT_MESH_DFU_SRV_INIT(&dfu_handlers, dfu_imgs,
							     ARRAY_SIZE(dfu_imgs));

void mesh_ota_target_init(void)
{
	bt_mesh_blob_io_flash_init(&blob_flash_stream, 0, 0);
}
#endif
