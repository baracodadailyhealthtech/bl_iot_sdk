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
#include "btblecontroller_port_os.h"
#include "FreeRTOS.h"
#include "rtos_al.h"
#include "task.h"

__attribute__((weak)) int btblecontroller_task_new(btblecontroller_TaskFunction_t taskFunction,const char *name, int stack_size, void *arg, int prio,void *taskHandler)
{
    return rtos_task_create( (rtos_task_fct)taskFunction, name, 0, stack_size, arg, prio,(rtos_task_handle * const)taskHandler);
}

__attribute__((weak)) void btblecontroller_task_delete(uint32_t taskHandler)
{
    rtos_task_delete((TaskHandle_t)taskHandler);
}

__attribute__((weak)) int btblecontroller_queue_new(uint32_t size, uint32_t max_msg,btblecontroller_QueueHandle_t *queue)
{
    return rtos_queue_create(max_msg,size,queue);
}

__attribute__((weak)) void btblecontroller_queue_free(btblecontroller_QueueHandle_t q)
{
    rtos_queue_delete(q);
}

__attribute__((weak)) int btblecontroller_queue_send(btblecontroller_QueueHandle_t q, void *msg, uint32_t size, uint32_t timeout)
{
    int status = rtos_queue_write(q, msg, timeout,false);
    if(status != 0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

__attribute__((weak)) int btblecontroller_queue_recv(btblecontroller_QueueHandle_t q, void *msg, uint32_t timeout)
{
    int status = rtos_queue_read(q, msg, timeout,false);
    if(status == 0)
    {
       return 1;
    }
    else
    {
       return 0;
    }
}

__attribute__((weak)) int btblecontroller_queue_send_from_isr(btblecontroller_QueueHandle_t q, void *msg, uint32_t size)
{
    rtos_queue_write(q, msg, 0, true);
    return 0;
}

__attribute__((weak)) int btblecontroller_xport_is_inside_interrupt()
{
    return HalIsIntActive();
}

__attribute__((weak)) void btblecontroller_task_delay(uint32_t ms)
{
    rtos_task_suspend(ms);
}