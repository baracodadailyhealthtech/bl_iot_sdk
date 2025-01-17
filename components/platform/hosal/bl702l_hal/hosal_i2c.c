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

#include "bl702l.h"
#include "bl702l_i2c.h"
#include "bl702l_clock.h"
#include "bl702l_glb.h"
#include "hosal_i2c.h"
#include "blog.h"

static void hosal_i2c_adjust_clock(uint32_t scl_freq)
{
    uint32_t div = 1;
    uint32_t clock = SystemCoreClockGet()/(GLB_Get_BCLK_Div()+1);
    uint32_t phase, phase0, phase1, phase2, phase3;

    while (1) {
        phase = (clock + scl_freq / 2) / scl_freq - 4;
        phase0 = (phase + 4) / 8;
        phase2 = (phase * 3 + 4) / 8;
        phase3 = (phase + 4) / 8;
        phase1 = phase - (phase0 + phase2 + phase3);
        if (phase0 > 255 || phase1 > 255 || phase2 > 255 || phase3 > 255) {
            div <<= 1;
            clock >>= 1;
        } else {
            break;
        }
    }

    GLB_Set_I2C_CLK(1, GLB_I2C_CLK_SRC_BCLK, div - 1);
}

int hosal_i2c_init(hosal_i2c_dev_t *i2c)
{
    GLB_GPIO_Type gpiopins[2];

    if (NULL == i2c || i2c->port != 0) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_I2C);
    GLB_Set_I2C_CLK(1, GLB_I2C_CLK_SRC_BCLK, 0);
    I2C_SetSclSync(i2c->port, 0);

    gpiopins[0] = i2c->config.scl;
    gpiopins[1] = i2c->config.sda;

    GLB_GPIO_Func_Init(GPIO_FUN_I2C, gpiopins, sizeof(gpiopins) / sizeof(gpiopins[0]));

    return 0;
}

int hosal_i2c_master_send(hosal_i2c_dev_t *i2c, uint16_t dev_addr, const uint8_t *data,
                            uint16_t size, uint32_t timeout)
{
    I2C_Transfer_Cfg i2c_cfg_send = {
        .slaveAddr = 0,
        .slaveAddr10Bit = DISABLE,
        .stopEveryByte = DISABLE,
        .subAddrSize = 0,
        .subAddr = 0x00,
        .dataSize = 0,
        .data = NULL,
        .clk = 0,
    };

    if (NULL == i2c || i2c->port != 0 || NULL == data || size < 1) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    i2c_cfg_send.clk = i2c->config.freq;
    i2c_cfg_send.slaveAddr10Bit = (i2c->config.address_width == HOSAL_I2C_ADDRESS_WIDTH_10BIT) ? ENABLE : DISABLE;
    i2c_cfg_send.slaveAddr = dev_addr;
    i2c_cfg_send.data = (uint8_t *)data;
    i2c_cfg_send.dataSize = size;

    hosal_i2c_adjust_clock(i2c->config.freq);
    return I2C_MasterSendBlocking(i2c->port, &i2c_cfg_send);
}

int hosal_i2c_master_recv(hosal_i2c_dev_t *i2c, uint16_t dev_addr, uint8_t *data,
                            uint16_t size, uint32_t timeout)
{
    I2C_Transfer_Cfg i2c_cfg_recv = {
        .slaveAddr = 0,
        .slaveAddr10Bit = DISABLE,
        .stopEveryByte = DISABLE,
        .subAddrSize = 0,
        .subAddr = 0x00,
        .dataSize = 0,
        .data = NULL,
        .clk = 0,
    };

    if (NULL == i2c || i2c->port != 0 || NULL == data || size < 1) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    i2c_cfg_recv.clk = i2c->config.freq;
    i2c_cfg_recv.slaveAddr10Bit = (i2c->config.address_width == HOSAL_I2C_ADDRESS_WIDTH_10BIT) ? ENABLE : DISABLE;
    i2c_cfg_recv.slaveAddr = dev_addr;
    i2c_cfg_recv.data = (uint8_t *)data;
    i2c_cfg_recv.dataSize = size;

    hosal_i2c_adjust_clock(i2c->config.freq);
    return I2C_MasterReceiveBlocking(i2c->port, &i2c_cfg_recv);
}

int hosal_i2c_slave_send(hosal_i2c_dev_t *i2c, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (NULL == i2c || i2c->port != 0 || NULL == data || size < 1) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    blog_error("not support now!\r\n");

    return -1;
}

int hosal_i2c_slave_recv(hosal_i2c_dev_t *i2c, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (NULL == i2c || i2c->port != 0 || NULL == data || size < 1) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    blog_error("not support now!\r\n");

    return -1;
}

int hosal_i2c_mem_write(hosal_i2c_dev_t *i2c, uint16_t dev_addr, uint32_t mem_addr,
                          uint16_t mem_addr_size, const uint8_t *data, uint16_t size,
                          uint32_t timeout)
{
    I2C_Transfer_Cfg i2c_cfg_send = {
        .slaveAddr = 0,
        .slaveAddr10Bit = DISABLE,
        .stopEveryByte = DISABLE,
        .subAddrSize = 0,
        .subAddr = 0x00,
        .dataSize = 0,
        .data = NULL,
        .clk = 0,
    };

    if (NULL == i2c || i2c->port != 0 || NULL == data || size < 1) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    i2c_cfg_send.clk = i2c->config.freq;
    i2c_cfg_send.slaveAddr10Bit = (i2c->config.address_width == HOSAL_I2C_ADDRESS_WIDTH_10BIT) ? ENABLE : DISABLE;
    i2c_cfg_send.slaveAddr = dev_addr;
    i2c_cfg_send.subAddr = mem_addr;
    i2c_cfg_send.subAddrSize = mem_addr_size;
    i2c_cfg_send.data = (uint8_t *)data;
    i2c_cfg_send.dataSize = size;

    hosal_i2c_adjust_clock(i2c->config.freq);
    return I2C_MasterSendBlocking(i2c->port, &i2c_cfg_send);
}

int hosal_i2c_mem_read(hosal_i2c_dev_t *i2c, uint16_t dev_addr, uint32_t mem_addr,
                         uint16_t mem_addr_size, uint8_t *data, uint16_t size,
                         uint32_t timeout)
{
    I2C_Transfer_Cfg i2c_cfg_recv = {
        .slaveAddr = 0,
        .slaveAddr10Bit = DISABLE,
        .stopEveryByte = DISABLE,
        .subAddrSize = 0,
        .subAddr = 0x00,
        .dataSize = 0,
        .data = NULL,
        .clk = 0,
    };

    if (NULL == i2c || i2c->port != 0 || NULL == data || size < 1) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    i2c_cfg_recv.clk = i2c->config.freq;
    i2c_cfg_recv.slaveAddr10Bit = (i2c->config.address_width == HOSAL_I2C_ADDRESS_WIDTH_10BIT) ? ENABLE : DISABLE;
    i2c_cfg_recv.slaveAddr = dev_addr;
    i2c_cfg_recv.subAddr = mem_addr;
    i2c_cfg_recv.subAddrSize = mem_addr_size;
    i2c_cfg_recv.data = (uint8_t *)data;
    i2c_cfg_recv.dataSize = size;

    hosal_i2c_adjust_clock(i2c->config.freq);
    return I2C_MasterReceiveBlocking(i2c->port, &i2c_cfg_recv);
}

int hosal_i2c_finalize(hosal_i2c_dev_t *i2c)
{
    if (NULL == i2c || i2c->port != 0) {
        blog_error("parameter is error!\r\n");
        return -1;
    }

    I2C_Disable(i2c->port);

    return 0;
}
