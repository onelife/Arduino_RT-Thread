/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     onelife      To deal with Arduino build process
 */

#if defined(ARDUINO_ARCH_SAM)
    #include "../../libcpu/arm/cortex-m3/cpuport.c"

#elif defined(ARDUINO_ARCH_SAMD)
    #include "../../libcpu/arm/cortex-m0/cpuport.c"

#else
    #error "Unsupported architecture!"
#endif
