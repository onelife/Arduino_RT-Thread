/***************************************************************************//**
 * @file    rtt.h
 * @brief   Arduino RT-Thread library header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __RTT_H__
#define __RTT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "include/rtthread.h"
#include "include/rthw.h"

#ifdef DFS_USING_POSIX
#include "components/dfs/include/dfs_file.h"
#include "components/libc/compilers/common/extension/sys/unistd.h"
#endif /* DFS_USING_POSIX */

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

class RT_Thread {
 public:
    void begin(void) __attribute__((noreturn));

 private:
    char rtt_heap[CONFIG_HEAP_SIZE];
};

extern RT_Thread RT_T;

#endif

#endif /*__RTT_H__ */
