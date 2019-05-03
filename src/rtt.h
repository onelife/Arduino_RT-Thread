/***************************************************************************//**
 * @file    rtt.h
 * @brief   Arduino RT-Thread library header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __RTT_H__
#define __RTT_H__

#include "include/rtthread.h"
#ifdef RT_USING_DFS
# include "components/dfs/include/dfs_posix.h"
#endif


class RT_Thread {
 public:
    void begin(void) __attribute__((noreturn));

 private:
    char rtt_heap[CONFIG_HEAP_SIZE];
};

extern RT_Thread RT_T;

#endif /*__RTT_H__ */
