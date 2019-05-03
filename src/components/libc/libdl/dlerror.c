/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Change Logs:
 * Date           Author      Notes
 * 2010-11-17     yi.qiu      first version
 */

#include "include/rtthread.h"

#ifdef RT_USING_MODULE


const char *dlerror(void) {
    return "TODO";
}
RTM_EXPORT(dlerror)

#endif /* RT_USING_MODULE */
