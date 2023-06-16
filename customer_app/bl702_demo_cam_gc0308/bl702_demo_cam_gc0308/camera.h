// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAMERA_PF_RGB565 = 0,       //!< RGB, 2 bytes per pixel (not implemented)
    CAMERA_PF_YUV422 = 1,       //!< YUYV, 2 bytes per pixel (not implemented)
    CAMERA_PF_GRAYSCALE = 2,    //!< 1 byte per pixel
    CAMERA_PF_JPEG = 3,         //!< JPEG compressed
} camera_pixelformat_t;

typedef enum {
    CAMERA_FS_QQVGA = 4,     //!< 160x120
    CAMERA_FS_QVGA = 8,      //!< 320x240
    CAMERA_FS_VGA = 10,      //!< 640x480
    CAMERA_FS_SVGA = 11,     //!< 800x600
} camera_framesize_t;

typedef enum {
    CAMERA_NONE = 0,
    CAMERA_UNKNOWN = 1,
    CAMERA_OV7725 = 7725,
    CAMERA_OV2640 = 2640,
    CAMERA_MT9D111 = 0x9d11,
    CAMERA_BF2013 = 0x2013,//24Pin
    CAMERA_BF3703 = 0x3703,//18Pin
    CAMERA_BF3A03 = 0x3a03,
    CAMERA_GC0328 = 0x0328,//18Pin
} camera_model_t;

typedef struct {
    camera_pixelformat_t pixel_format;
    camera_framesize_t frame_size;

    int jpeg_quality;
} camera_config_t;

#define ESP_ERR_CAMERA_BASE 0x20000
#define ESP_ERR_CAMERA_NOT_DETECTED             (ESP_ERR_CAMERA_BASE + 1)
#define ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE (ESP_ERR_CAMERA_BASE + 2)
#define ESP_ERR_CAMERA_NOT_SUPPORTED            (ESP_ERR_CAMERA_BASE + 3)

const rt_camera_desc* camera_module_init(void);

#ifdef __cplusplus
}
#endif
