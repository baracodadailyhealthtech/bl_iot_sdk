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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensor.h"
#include "sccb.h"
#include "camera.h"

const rt_camera_desc* camera_module_init(void)
{
    extern const rt_camera_desc __rt_camera_gc0308_desc;
    const rt_camera_desc *desc = &__rt_camera_gc0308_desc;

    if((desc->probe() == 0)) {
        printf("[camera_module_init] Resetting camera module [%p, %s, 0x%02x]\r\n", desc, desc->name, desc->addr);
        desc->reset();
        return desc;
    } else {
        return NULL;
    }
}
