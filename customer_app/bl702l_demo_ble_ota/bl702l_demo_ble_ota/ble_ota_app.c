#include <FreeRTOS.h>
#include <task.h>
#include <stdlib.h>
#include <hosal_ota.h>
#include "bl_flash.h"
#include "hal_boot2.h"
#include "conn.h"
#include "conn_internal.h"
#include "gatt.h"
#include "hci_core.h"
#include "btble_lib_api.h"
#include "bluetooth.h"
#include "hci_driver.h"
#include "queue.h"
#include "oad.h"
#ifdef CONFIG_BT_STACK_CLI 
#include "ble_cli_cmds.h"
#endif
#include "oad_main.h"
#include "oad_service.h"
#include "ble_ota_app.h"
#include "ble_ota_config.h"

typedef enum _MSG_CMD_TYPE{
    MSG_START_SCAN,
	MSG_STOP_SCAN,
	MSG_START_CONN,
	MSG_DISCOVER_START,
	MSG_SUBSCRIBE_START,
	MSG_OTA_IMAG_INFO,
	MSG_OTA_IMAG_IDENTITY,
	MSG_OTA_IMAG_BLOCK_REQ,
	MSG_OTA_IMAG_UPGRD_END,
    MSG_MAX_TYPE,
}MSG_CMD_TYPE;

enum{
    OTA_CMD_IMAG_IDENTITY = 0x00,
    OTA_CMD_IMAG_BLOCK_REQ,
    OTA_CMD_IMAG_BLOCK_RESP,
    OTA_CMD_IMAG_UPGRD_END,
    OTA_CMD_IMAG_INFO,
};

struct slave_msg_struct{
	u8_t isotastart;
	u8_t isotasucess;
	u16_t ota_wr_hdl;    
	u16_t ota_notify_hdl;    
	u16_t ota_ccc_hdl; 
	u16_t index; 
	u16_t mtu; 
	u16_t left_size; 
	u16_t oDataLen;
	u32_t file_offset;
	u32_t read_count;
	u32_t read_offset;
	u32_t starttick;
	u32_t endtick;
	struct bt_conn * ble_conn;
	struct oad_block_rsp_t block_rsp;
	u8_t recvbuf[MAX_RECV_BUF_SIZE];
	u8_t sendbuf[MAX_RECV_BUF_SIZE];
	u8_t databuf[MAX_DATA_BUF_SIZE];
};

struct msg_cmd {
	u8_t index;
    MSG_CMD_TYPE type;
	bt_addr_le_t addr;
};

static QueueHandle_t test_msg_queue = NULL; 
static u8_t conn_total_nums = 0;
static struct bt_gatt_discover_params ble_ota_discover_params;
static struct bt_gatt_subscribe_params ble_ota_subscribe_params[MAX_SLAVE_NUMS];
static struct slave_msg_struct slaveinfo[MAX_SLAVE_NUMS];
uint32_t OTA_IMAGE_SIZE = 0;
uint32_t OTA_PARTITION_START_ADDR = 0;
uint32_t OTA_IMAGE_START_ADDR = 0;
#define OTA_IMAGE_SIZE_OFFSET 0x1000
#define OTA_IMAGE_OFFSET 0x1004

static u8_t ble_ota_get_slave_index(struct bt_conn *conn)
{
	u8_t i = 0;
	
	for(i = 0; i < MAX_SLAVE_NUMS; i++)
	{
		if (slaveinfo[i].ble_conn == conn)
		{
			break;
		}
	}

	return i;
}

static void ble_ota_send_msg(MSG_CMD_TYPE msgtype, u8_t index)
{
	struct msg_cmd *cmd; 
    cmd = k_malloc(sizeof(struct msg_cmd)); 
    cmd->type = msgtype;
    cmd->index = index;
	//printf("send_msg msgtype= %d, index = %d \r\n", msgtype, index);
    if (pdFALSE == xQueueSend(test_msg_queue, &cmd, 0)) 
	{
	    printf("------------------send_msg fail \r\n");
	}
}

static void ble_ota_tx_mtu_size(struct bt_conn *conn, u8_t err,
			  struct bt_gatt_exchange_params *params)
{
    k_free(params);
    if(!err)
    {
         printf("ble ota echange mtu size success, mtu size: %d\n", bt_gatt_get_mtu(conn));
    }
    else
    {
         printf("ble ota echange mtu size failure, err: %d\n", err);
    }
	
	u8_t index = ble_ota_get_slave_index(conn);
	ble_ota_send_msg(MSG_DISCOVER_START, index);
}

static void ble_ota_connected(struct bt_conn *conn, u8_t err)
{
    char addr[30];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		printf("Failed to connect to %s (%u)\r\n", addr, err);
		ble_ota_send_msg(MSG_START_SCAN, 0);
		return;
	}
	printf("BLE Connected: %s\r\n", addr);
	
	u8_t i = 0;
	for(i = 0; i < MAX_SLAVE_NUMS; i++)
	{
		if (slaveinfo[i].ble_conn == NULL)
		{
			slaveinfo[i].ble_conn = conn;
			break;
		}
	}
	
	conn_total_nums++;
	int tx_octets = 0x00fb;    
	int tx_time = 0x0848;    
	int ret = -1;    

	ret = bt_le_set_data_len(slaveinfo[i].ble_conn, tx_octets, tx_time);    
	if (!ret) {        
		printf("ble set data length success\r\n");    
	} else {        
		printf("ble set data length failure, err: %d\r\n", ret);
	}

	//exchange mtu size after connected.
	struct bt_gatt_exchange_params *exchg_mtu = k_malloc(sizeof(struct bt_gatt_exchange_params));
	exchg_mtu->func = ble_ota_tx_mtu_size;
	ret = -1;    
	ret = bt_gatt_exchange_mtu(slaveinfo[i].ble_conn, exchg_mtu);
	if (!ret) {
		printf("ble ota exchange mtu size pending.\n");
	} else {
		k_free(exchg_mtu);
		printf("ble ota exchange mtu size failure, err: %d\n", ret);
		ble_ota_send_msg(MSG_DISCOVER_START, i);
	}	
} 

static void ble_ota_disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[30];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printf("Disconnected: %s (reason 0x%02x)\r\n", addr, reason);
	conn_total_nums--;
	u8_t index = ble_ota_get_slave_index(conn);
	memset(&slaveinfo[index], 0, sizeof(struct slave_msg_struct));
	ble_ota_send_msg(MSG_START_SCAN, 0);
}

static struct bt_conn_cb ble_ota_conn_callbacks = {
	.connected = ble_ota_connected,
	.disconnected = ble_ota_disconnected,
}; 

static bool ble_ota_parse_data_cb(struct bt_data *data, void *user_data)
{
    char *name = user_data;
    u8_t len;

    switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        len = (data->data_len > 30 - 1)?(30 - 1):(data->data_len);
        memcpy(name, data->data, len);
        return false;
    default:
        return true;
    }
}

static void ble_ota_scan_cb(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
			 struct net_buf_simple *ad)
{
	char dev[30];
    char name[30];
	
	bt_addr_le_to_str(addr, dev, sizeof(dev));
	if ((type == BT_LE_ADV_IND)||(type == BT_LE_ADV_SCAN_RSP))
	{
		memset(name, 0, sizeof(name));
		bt_data_parse(ad, ble_ota_parse_data_cb, name);
		if (NULL != strstr(name, BLE_OTA_DEV_NAME))
		{
			struct bt_conn *conn = NULL;
			conn = bt_conn_lookup_state_le(addr, BT_CONN_CONNECTED);
			if (conn)
			{
				printf("already connect\r\n");
				printf("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i NAME=%s\r\n",
	       										dev, type, ad->len, rssi,name);
				bt_conn_unref(conn);
				return;
			}
			if (conn_total_nums >= MAX_SLAVE_NUMS)
			{
				printf("cur ble conn is max\r\n");
				return;
			}
			bt_le_scan_stop();
			printf("ble device_found ok\r\n");
			printf("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i NAME=%s\r\n",
												dev, type, ad->len, rssi,name);
			struct msg_cmd *cmd;
		    cmd = k_malloc(sizeof(struct msg_cmd));
		    cmd->type = MSG_START_CONN;
			memcpy(&(cmd->addr), addr, sizeof(bt_addr_le_t));
			xQueueSend(test_msg_queue, &cmd, 0);
		}
	}
}

static void ble_ota_start_scan(void)
{
	int err;
 	struct bt_le_scan_param scan_param;

	scan_param.type=BT_LE_SCAN_TYPE_ACTIVE;
	scan_param.filter_dup=0;
	scan_param.interval=0x80;//0x80;
	scan_param.window=0x20;
	err = bt_le_scan_start(&scan_param, ble_ota_scan_cb);
	if (err) {
		printf("Scanning failed to start (err %d)\r\n", err);
		return;
	}
	printf("ble scan started\n");
}

static u8_t ble_ota_notify_func(struct bt_conn *conn,
                        struct bt_gatt_subscribe_params *params,
                        const void *data, u16_t length)
{
    //printf("test_notify_func complete\r\n");
    if (!params->value) {
        printf("Unsubscribed\r\n");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (length) 
	{
		#if 0
        uint8_t *recv_buffer;
		recv_buffer=pvPortMalloc(sizeof(uint8_t)*length);
		memcpy(recv_buffer, data, length);
		printf("ble notification=");
		for (int i = 0; i < length; i++)
		{
			printf("0x%x ",recv_buffer[i]);
		}
		k_free(recv_buffer);
		printf("\n");	
		#endif
		u8_t index = ble_ota_get_slave_index(conn);
		//memset(slaveinfo[index].recvbuf, 0, MAX_RECV_BUF_SIZE);
		memcpy(slaveinfo[index].recvbuf, data, length);
		u8_t infotype = slaveinfo[index].recvbuf[0];
		if (infotype == OTA_CMD_IMAG_BLOCK_REQ)
		{
		    ble_ota_send_msg(MSG_OTA_IMAG_BLOCK_REQ, index);
		}
		else if (infotype == OTA_CMD_IMAG_INFO)
		{
		    ble_ota_send_msg(MSG_OTA_IMAG_IDENTITY, index);
		}
		else if (infotype == OTA_CMD_IMAG_UPGRD_END)
		{
			ble_ota_send_msg(MSG_OTA_IMAG_UPGRD_END, index);
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

static int ble_ota_subscribe(struct bt_conn *conn, u8_t index)
{
    int ret = -1;
	
    ble_ota_subscribe_params[index].ccc_handle = slaveinfo[index].ota_ccc_hdl;
    ble_ota_subscribe_params[index].value_handle = slaveinfo[index].ota_notify_hdl;
    ble_ota_subscribe_params[index].value = 1;
    ble_ota_subscribe_params[index].notify = ble_ota_notify_func;

    int err = bt_gatt_subscribe(conn, &ble_ota_subscribe_params[index]);
    if (err) {
        printf("Subscribe failed (err %d)\r\n", err);
		ret = -1;
    } else {
        printf("Subscribed sucess\r\n");
		ret = 0;
    }
	
	return ret;
}

static u8_t ble_ota_discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	struct bt_gatt_service_val *gatt_service = NULL;
	struct bt_gatt_chrc *gatt_chrc = NULL;
	struct bt_gatt_include *gatt_include = NULL;  
	char str[37];

	u8_t index = ble_ota_get_slave_index(conn);
	if (!attr) {
		printf("Discover complete\r\n");
		(void)memset(params, 0, sizeof(*params));
		ble_ota_send_msg(MSG_SUBSCRIBE_START, index);
		return BT_GATT_ITER_STOP;
	}
	if (params == NULL) {        
		printf("ble_discover_func_PARAMS == NULL\r\n");    
	}
	printf("[ATTRIBUTE] handle %d\r\n", attr->handle);

	switch (params->type) {
		case BT_GATT_DISCOVER_PRIMARY:            
		case BT_GATT_DISCOVER_SECONDARY:  
			gatt_service = attr->user_data;
		    bt_uuid_to_str(gatt_service->uuid, str, sizeof(str));
			printf("Service %s found: start handle %x, end_handle %x\r\n", str, attr->handle, gatt_service->end_handle);
			break;
		case BT_GATT_DISCOVER_CHARACTERISTIC:
            gatt_chrc = attr->user_data;
            bt_uuid_to_str(gatt_chrc->uuid, str, sizeof(str));
			printf("discover_func uuid type = %d\r\n", gatt_chrc->uuid->type);
            if (!bt_uuid_cmp(gatt_chrc->uuid, OTA_UUID_OAD_DATA_IN)) { 
                slaveinfo[index].ota_wr_hdl = gatt_chrc->value_handle; 
				printf("slaveinfo[index].ota_wr_hdl %d\r\n", slaveinfo[index].ota_wr_hdl);
            } else if (!bt_uuid_cmp(gatt_chrc->uuid, OTA_UUID_OAD_DATA_OUT)) {
                slaveinfo[index].ota_notify_hdl = gatt_chrc->value_handle;
                slaveinfo[index].ota_ccc_hdl= slaveinfo[index].ota_notify_hdl+1;
				printf("slaveinfo[index].ota_notify_hdl %d\r\n", slaveinfo[index].ota_notify_hdl);
            }
            break; 

        case BT_GATT_DISCOVER_INCLUDE: 
			gatt_include = attr->user_data;
		    bt_uuid_to_str(gatt_include->uuid, str, sizeof(str));
            break;

        default:
			bt_uuid_to_str(attr->uuid, str, sizeof(str));
            break;
	}
	
	return BT_GATT_ITER_CONTINUE;
}


static void ble_ota_client_task(void * para)
{
    struct msg_cmd* p_msg_cmd = NULL;

    while (1) {
        p_msg_cmd = NULL;
        xQueueReceive(test_msg_queue, &p_msg_cmd, portMAX_DELAY);
		if (p_msg_cmd)
		{
			switch(p_msg_cmd->type)
			{
				case MSG_OTA_IMAG_BLOCK_REQ:
				{
					if (slaveinfo[p_msg_cmd->index].ble_conn != NULL)
					{
						//printf("MSG_OTA_IMAG_BLOCK_REQ \r\n");
						//vTaskSuspendAll();
						u8_t i = p_msg_cmd->index, len = 0;
						struct oad_block_req_t* block_req = (struct oad_block_req_t*)(&(slaveinfo[i].recvbuf[1]));
						//printf("manu_code = 0x%04x\r\n", block_req->file_info.manu_code);
						//printf("file_ver = 0x%08x\r\n", (unsigned int)block_req->file_info.file_ver);
						//printf("\nfile_offset = 0x%08x\r\n", (unsigned int)block_req->file_offset);
						//printf("111 slaveinfo[i].file_offset = 0x%08x\r\n", (unsigned int)slaveinfo[i].file_offset);
						struct oad_block_rsp_t* block_rsp = &(slaveinfo[i].block_rsp);
						block_rsp->status = OAD_SUCC;
						memcpy(&(block_rsp->file_info), &block_req->file_info, sizeof(struct oad_file_info));
						
						if (slaveinfo[i].file_offset == block_req->file_offset)
						{
							block_rsp->file_offset = slaveinfo[i].file_offset;
							u8_t sendlen = 0;
							if ((slaveinfo[i].file_offset+slaveinfo[i].mtu) <= OTA_IMAGE_SIZE)
							{
								sendlen = slaveinfo[i].mtu;
							}
							else
							{
								sendlen = (OTA_IMAGE_SIZE-slaveinfo[i].file_offset);
							}
							slaveinfo[i].left_size = (MAX_DATA_BUF_SIZE - slaveinfo[i].index);
                            if (slaveinfo[i].left_size >= sendlen)
                        	{
                        		memcpy(&(slaveinfo[i].sendbuf[OAD_OPCODE_SIZE+OAD_BLK_RSP_DATA_OFFSET]), &(slaveinfo[i].databuf[slaveinfo[i].index]), sendlen);
								slaveinfo[i].index += sendlen;
                        	}
							else
							{
								slaveinfo[i].oDataLen = (sendlen - slaveinfo[i].left_size);
								memcpy(&(slaveinfo[i].sendbuf[OAD_OPCODE_SIZE+OAD_BLK_RSP_DATA_OFFSET]),&(slaveinfo[i].databuf[slaveinfo[i].index]),slaveinfo[i].left_size);
								slaveinfo[i].index += slaveinfo[i].left_size;
								if(slaveinfo[i].index == MAX_DATA_BUF_SIZE)
								{
									if ((slaveinfo[i].read_offset+MAX_DATA_BUF_SIZE) <= OTA_IMAGE_SIZE)
									{
										bl_flash_read(OTA_IMAGE_START_ADDR + slaveinfo[i].read_offset, slaveinfo[i].databuf, MAX_DATA_BUF_SIZE);
									}
									else
									{
										bl_flash_read(OTA_IMAGE_START_ADDR +slaveinfo[i].read_offset, slaveinfo[i].databuf, (OTA_IMAGE_SIZE-slaveinfo[i].read_offset));
									}
								}
								slaveinfo[i].read_count += 1;
			                    slaveinfo[i].read_offset = (slaveinfo[i].read_count * MAX_DATA_BUF_SIZE);
								slaveinfo[i].index = 0;
								u16_t index = (OAD_OPCODE_SIZE+OAD_BLK_RSP_DATA_OFFSET+slaveinfo[i].left_size); 
								memcpy(&(slaveinfo[i].sendbuf[index]), slaveinfo[i].databuf, slaveinfo[i].oDataLen);
								slaveinfo[i].index += slaveinfo[i].oDataLen;
							}
							block_rsp->data_size = sendlen;
                        	block_rsp->pdata = &(slaveinfo[i].sendbuf[OAD_OPCODE_SIZE+OAD_BLK_RSP_DATA_OFFSET]);
							slaveinfo[i].file_offset += sendlen;
						    //printf("sendlen=%d \r\n", sendlen);
							if (slaveinfo[i].file_offset == OTA_IMAGE_SIZE)
							{
								printf("master send ota data complete wait ota complete msg \r\n");
							}
							else if (slaveinfo[i].file_offset > OTA_IMAGE_SIZE)
							{
								printf("slaveinfo[i].file_offset > OTA_FILE_SIZE \r\n");
								bt_conn_disconnect(slaveinfo[i].ble_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
							}
						}
						else
						{
							printf("slaveinfo[i].file_offset != block_req->file_offset \r\n");
							bt_conn_disconnect(slaveinfo[i].ble_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
						}
						slaveinfo[i].sendbuf[0] = OTA_CMD_IMAG_BLOCK_RESP;
						len = OAD_BLK_RSP_DATA_OFFSET;
						memcpy(&(slaveinfo[i].sendbuf[1]), block_rsp, len);
						len = (OAD_OPCODE_SIZE+OAD_BLK_RSP_DATA_OFFSET+block_rsp->data_size);
						//xTaskResumeAll();
						int ret = -1;
						ret = bt_gatt_write_without_response(slaveinfo[i].ble_conn, slaveinfo[i].ota_wr_hdl, 
											slaveinfo[i].sendbuf, len, 0); 
						if (ret < 0) 
						{
	                         printf("MSG_OTA_IMAG_BLOCK_REQ Write error ret = %d \r\n", ret);
						}
					} 
				}
				break;

				case MSG_OTA_IMAG_UPGRD_END:
				{
					if (slaveinfo[p_msg_cmd->index].ble_conn != NULL)
					{
						printf("MSG_OTA_IMAG_UPGRD_END \r\n");
						struct oad_upgrd_end_t* upgrd_end = (struct oad_upgrd_end_t*)(&(slaveinfo[p_msg_cmd->index].recvbuf[1]));
						if (upgrd_end->status == OAD_SUCC)
						{
						    slaveinfo[p_msg_cmd->index].endtick = xTaskGetTickCount(); 
							printf("slave ota success \r\n");
							printf("ota endtick = %ld (ms) index = %d \r\n", (long int)slaveinfo[p_msg_cmd->index].endtick, p_msg_cmd->index);
							printf("ota total time = %ld (ms)\r\n", (long int)(slaveinfo[p_msg_cmd->index].endtick-slaveinfo[p_msg_cmd->index].starttick));
						}
						else
						{
							printf("slave ota fail status  = %d index = %d \r\n", upgrd_end->status, p_msg_cmd->index);
						}
						
						bt_conn_disconnect(slaveinfo[p_msg_cmd->index].ble_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
					}
				}
				break;
				
			    case MSG_OTA_IMAG_IDENTITY:
				{
					if (slaveinfo[p_msg_cmd->index].ble_conn != NULL)
					{
						printf("MSG_OTA_IMAG_IDENTITY \r\n");
						int ret = -1;
						u8_t i = p_msg_cmd->index, len = 0;
						struct oad_image_identity_t identity;
						
						slaveinfo[i].index = 0;
						slaveinfo[i].left_size = MAX_DATA_BUF_SIZE;
						slaveinfo[i].oDataLen = 0;
						slaveinfo[i].file_offset = 0;
						slaveinfo[i].mtu = bt_gatt_get_mtu(slaveinfo[i].ble_conn);
						printf("MSG_OTA_IMAG_IDENTITY  mtu = %d \r\n", slaveinfo[i].mtu);
						slaveinfo[i].mtu -= (3+OAD_OPCODE_SIZE+OAD_BLK_RSP_DATA_OFFSET);
						bl_flash_read(OTA_IMAGE_START_ADDR, slaveinfo[i].databuf, MAX_DATA_BUF_SIZE);
						slaveinfo[i].read_count = 1;
			            slaveinfo[i].read_offset = slaveinfo[i].read_count * MAX_DATA_BUF_SIZE; 
						slaveinfo[i].starttick = xTaskGetTickCount(); 
						printf("ota starttick = %ld (ms) index = %d \r\n", (long int)slaveinfo[i].starttick, i);
						identity.crc32 = 0;// unused
						identity.file_size = OTA_IMAGE_SIZE;
						identity.file_info.manu_code = 0x2c00;
						identity.file_info.file_ver = 0x16000000;
						//memset(slaveinfo[i].recvbuf, 0, MAX_RECV_BUF_SIZE);
						len = sizeof(struct oad_image_identity_t);
						slaveinfo[i].sendbuf[0] = OTA_CMD_IMAG_IDENTITY;
						memcpy(&(slaveinfo[i].sendbuf[1]), &identity, len);
						len += 1; 
						ret = bt_gatt_write_without_response(slaveinfo[i].ble_conn, slaveinfo[i].ota_wr_hdl, 
											slaveinfo[i].sendbuf, len, 0);
						if (ret < 0) 
						{
	                         printf("MSG_OTA_IMAG_IDENTITY Write error ret = %d \r\n", ret);
						}
					}
				}
				break;
				
			    case MSG_OTA_IMAG_INFO:
				{
					if (slaveinfo[p_msg_cmd->index].ble_conn != NULL)
					{
						printf("OTA start get version index = %d \r\n", p_msg_cmd->index); 
						u8_t i = p_msg_cmd->index;
						int ret = -1;
						u8_t data[1];
						data[0] = OTA_CMD_IMAG_INFO;
						ret = bt_gatt_write_without_response(slaveinfo[i].ble_conn, slaveinfo[i].ota_wr_hdl, 
											data, 1, 0);
						if (ret < 0)
						{
	                         printf("MSG_OTA_IMAG_INFO Write error ret = %d \r\n", ret);
						}
					}
				}
				break;
				
			    case MSG_START_CONN:
				{
					struct bt_le_conn_param param = {
							.interval_min =  6,
							.interval_max =  6,
							.latency = 0,
							.timeout = 500,
					};
					struct bt_conn *conn = NULL;
					conn = bt_conn_create_le(&(p_msg_cmd->addr), &param);
					if(!conn){ 
			        	printf("BLE Connection failed restart ble scan\r\n");
						ble_ota_start_scan();
			        }else{
			        	printf("Connection sucess\r\n");
			        } 
				}
				break;

				case MSG_DISCOVER_START:
				{
					if (slaveinfo[p_msg_cmd->index].ble_conn != NULL)
					{
						int err; 
						ble_ota_discover_params.uuid = NULL;
						ble_ota_discover_params.func = ble_ota_discover_func;
						ble_ota_discover_params.start_handle = 0x0001;
						ble_ota_discover_params.end_handle = 0xffff;
						ble_ota_discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC; 
						err = bt_gatt_discover(slaveinfo[p_msg_cmd->index].ble_conn, &ble_ota_discover_params);
						if (err) {
							printf("Discover failed(err %d)\r\n", err);  
							bt_conn_disconnect(slaveinfo[p_msg_cmd->index].ble_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
						} else {         
							printf("Discover pending\r\n");    
						}
					}
				}
				break;

				case MSG_SUBSCRIBE_START:
				{
					if (slaveinfo[p_msg_cmd->index].ble_conn != NULL)
					{
					    int ret = -1;
						
						ret = ble_ota_subscribe(slaveinfo[p_msg_cmd->index].ble_conn, p_msg_cmd->index);
						if (!ret)
						{
							ble_ota_send_msg(MSG_OTA_IMAG_INFO, p_msg_cmd->index);
							if (conn_total_nums < MAX_SLAVE_NUMS) 
							{
								ble_ota_start_scan();
							}
						}
						else
						{
						    bt_conn_disconnect(slaveinfo[p_msg_cmd->index].ble_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
						}
					}
				}
				break;
				
				case MSG_START_SCAN:
				{
					ble_ota_start_scan();
				}
				break;
				
				case MSG_STOP_SCAN:
				{
					bt_le_scan_stop();
					printf("ble scan stop\r\n");
				}
				break;
				
				default:
					break;
			}
			k_free(p_msg_cmd); 
		}
    } 
}

void ble_ota_client_app_start(void)
{
	bt_conn_cb_register(&ble_ota_conn_callbacks);
    test_msg_queue = xQueueCreate(MAX_MESSAGE_NUMS, sizeof(void *));
	if (test_msg_queue == NULL)
	{
		printf("Ram is not enough \r\n");
		return;
	}

    uint32_t ota_partition_size = 0;
    if(hal_boot2_partition_addr_inactive("FW", &OTA_PARTITION_START_ADDR, &ota_partition_size))
    {
        printf("get fw partition failed\r\n");
        return;
    }

    if(bl_flash_read(OTA_IMAGE_SIZE_OFFSET + OTA_PARTITION_START_ADDR, (uint8_t *)&OTA_IMAGE_SIZE, sizeof(OTA_IMAGE_SIZE)))
    {
        printf("read ota image size from flash failed\r\n");
        return;
    }
    else
        printf("ota partition start addr 0x%lx, ota image size is %lu\r\n", OTA_PARTITION_START_ADDR, OTA_IMAGE_SIZE);

    OTA_IMAGE_START_ADDR = OTA_PARTITION_START_ADDR + OTA_IMAGE_OFFSET;
	memset(slaveinfo, 0, sizeof(slaveinfo));
	xTaskCreate(ble_ota_client_task, (char *)"bleOtaClient", 512, NULL, 20, NULL); //stack depth is 512*4=2048 bytes
	ble_ota_send_msg(MSG_START_SCAN, 0);
}

#if ROLE_SELECT == 0
#if defined(CONFIG_BT_OAD_SERVER)
bool app_check_oad(u32_t cur_file_ver, u32_t new_file_ver)
{
    //App layer decides whether to do oad according to file version
    /*if(new_file_ver > cur_file_ver)
        return true;
    else
        return false;*/
    return true;
}
#endif

void ble_adv_start(void)
{
	uint8_t mac[6];
	char macstr[13];
	char adv_name[32] = BLE_OTA_DEV_NAME; 
    #if 0
	bt_addr_le_t addr;
	bt_get_local_public_address(&addr);
	mac[0] = addr.a.val[5];
	mac[1] = addr.a.val[4];
	mac[2] = addr.a.val[3];
	mac[3] = addr.a.val[2];
	mac[4] = addr.a.val[1];
	mac[5] = addr.a.val[0];
	memset(adv_name, 0, 32);
	strcat(adv_name, "702l_"); 
	snprintf(macstr, sizeof(macstr), "%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2], mac[3], mac[4], mac[5]);
	strcat(adv_name, macstr);
	printf("adv_name=%s \r\n", adv_name); 
	#endif
	int err;
	struct bt_le_adv_param param;
	param.id = 0;
    param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
    param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;
    param.options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME);
	
    printf("Bluetooth Advertising start\r\n");
	struct bt_data ad_test[2] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE,  adv_name, strlen(adv_name)),
    };
    printf("adv_name=%s \r\n", adv_name); 
    #if 0
	ad_test[1].type = BT_DATA_NAME_COMPLETE;
	ad_test[1].data = (uint8_t *)adv_name;      
	ad_test[1].data_len = strlen(adv_name);
    #endif
	err = bt_le_adv_start(&param, ad_test, ARRAY_SIZE(ad_test), &ad_test[0], 1);
    if (err) {
        printf("Advertising failed to start (err %d)\r\n", err);
    }else{
         printf("Advertising sucess to start (err %d)\r\n", err); 
    }
}
#endif

void bt_enable_cb(int err)
{
    if (!err) {
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);

#ifdef CONFIG_BT_STACK_CLI 
        ble_cli_register();
#endif

#if ROLE_SELECT  //master role
    ble_ota_client_app_start();
#else //slave role
#if defined(CONFIG_BT_OAD_SERVER)
    oad_service_enable(app_check_oad);
#endif
#endif
    }
}

void ble_stack_start(void)
{       
     // Initialize BLE controller
    btble_controller_init(configMAX_PRIORITIES - 1);
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
}
