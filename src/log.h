/***************************************************************************//**
 * @file    rtt.h
 * @brief   Arduino RT-Thread library logger header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __LOG_H__
#define __LOG_H__

#include "include/rtthread.h"
#ifdef RT_USING_ULOG
# include "components/utilities/ulog/ulog.h"
#else
# warning "Logger module is not enabled"
#endif

#endif /*__LOG_H__ */
