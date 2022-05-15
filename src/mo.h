/***************************************************************************//**
 * @file    mo.h
 * @brief   Arduino RT-Thread library module header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __MO_H__
#define __MO_H__

#include "include/rtthread.h"
#ifdef RT_USING_ULOG
# include "components/utilities/ulog/ulog.h"
#else
# warning "Logger module is not enabled"
#endif
#ifdef RT_USING_MODULE
# include "components/libc/posix/libdl/dlmodule.h"
# include "components/libc/posix/libdl/dlfcn.h"
#endif

#endif /*__MO_H__ */
