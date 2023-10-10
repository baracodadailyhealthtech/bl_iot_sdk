#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <sccb.h>
#include <camera.h>
#include <bl_cam.h>
#include <bl_gpio.h>
#include <vfs.h>
#include <hal/soc/uart.h>
#include <device/vfs_uart.h>
#include "bl702_uart.h"

static void enable_camera_module()
{
#if 0
#define CAM_GPIO_PWR (15)

    /*Disable Power Down Mode*/
    bl_gpio_output_set(CAM_GPIO_PWR, 0);
    bl_gpio_enable_output(CAM_GPIO_PWR, 0, 0);
#endif
}

static int app_pic_jpg_get(uint8_t **pic, int *len, int wait)
{
    uint8_t *ptr1, *ptr2;
    static uint32_t counter = 0;
    uint32_t len1 = 0, len2 = 0, frames;

retry:
    if (0 == bl_cam_frame_get(&frames, &ptr1, &len1, &ptr2, &len2)) {
        /*成功获取到图片*/
        printf("[YUV] %d frames %u, frame ptr1 %p len %u, ptr2 %p len %u; counter %u\r\n",
                (int)xTaskGetTickCount(),
                (unsigned int)frames,
                ptr1,
                (unsigned int)len1,
                ptr2,
                (unsigned int)len2,
                (unsigned int)counter++
        );
    } else {
        if (wait) {
            vTaskDelay(2);
            goto retry;
        }
    }

    if (0 == len1) {
        return -1;
    }
    *pic = ptr1;
    *len = len1;
    return 0;
}

static void app_pic_jpg_release()
{
    bl_cam_frame_pop();
}

static int app_pic_hardware_init()
{
    const rt_camera_desc *desc;

    /*enable_24mhz*/
    bl_cam_enable_24MRef();
    vTaskDelay(2);
    enable_camera_module();
    vTaskDelay(2);

    SCCB_Init();
    desc = camera_module_init();
    if (NULL == desc) {
        return -1;
    }
    printf("camera resolution: %d x %d\r\n", desc->width, desc->height);
    bl_cam_config_update(50);
    /*enable CAM module*/
    bl_cam_init(1, desc);

    return 0;
}

static void app_pic_encoder(uint8_t *pic, int size)
{
    /*这个函数里面可以改成图片识别、图片压缩的代码*/
    printf("Encoder PIC @%p with len %d Bytes\r\n", pic, size);

#if 0
    int len;
    while(size > 0) {
        len = size >= 256 ? 256 : size;
        UART_SendData(0, pic, len);
        arch_delay_ms(10);
        pic += len;
        size -= len;
    }
    printf("\r\ndump done\r\n");
    vTaskDelay(3000);
#endif
}

/*此任务的STACK，目前是16K*/
void cam_task_entry(void *pvParameters)
{
#if !defined(CFG_BLE_ENABLE)
    uint8_t *pic_addr = 0; //获取到的图片的所在的内存地址
    int pic_size = 0; //获取到的图片大小
#endif

    if (app_pic_hardware_init()) {
        printf("[APP] PIC harware init failed\r\n");
        goto done;
    }

#if !defined(CFG_BLE_ENABLE)
    while (1) {
        /*获取图片，图片的格式(YUV422 YUV420等)取决于sensor的配置*/
        app_pic_jpg_get(&pic_addr, &pic_size, 1);

        /*处理图片，这里不需要复制一份图片，可以直接使用内存中的图片*/
        app_pic_encoder(pic_addr, pic_size);

        /*释放图片所占用的内存资源，以便硬件可以继续采集图片*/
        app_pic_jpg_release();

        vTaskDelay(5);
    }
#endif


done:
    puts("[THREAD] cam task is done\r\n");
    vTaskDelete(NULL);
}
