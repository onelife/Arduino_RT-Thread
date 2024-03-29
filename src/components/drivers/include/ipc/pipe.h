/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#ifndef PIPE_H__
#define PIPE_H__

#include "include/rtthread.h"

/**
 * Pipe Device
 */

struct rt_pipe_device
{
    struct rt_device parent;
    rt_bool_t is_named;

    /* ring buffer in pipe device */
    struct rt_ringbuffer *fifo;
    rt_uint16_t bufsz;

    rt_uint8_t readers;
    rt_uint8_t writers;

    rt_wqueue_t reader_queue;
    rt_wqueue_t writer_queue;

    struct rt_mutex lock;
};
typedef struct rt_pipe_device rt_pipe_t;

rt_pipe_t *rt_pipe_create(const char *name, int bufsz);
int rt_pipe_delete(const char *name);

#endif /* PIPE_H__ */
