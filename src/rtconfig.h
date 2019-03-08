/***************************************************************************//**
 * @file    rtt.h
 * @brief   Arduino RT-Thread library config
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/* Porting Options */
#define CONFIG_ARDUINO
#define CONFIG_TICK_PER_SECOND          (1000)  /* Arduino */
#ifndef CONFIG_HEAP_SIZE
    #define CONFIG_HEAP_SIZE            (12 * 1024)
#endif
#ifndef CONFIG_PRIORITY_MAX
    #define CONFIG_PRIORITY_MAX         (3)
#endif
#ifndef CONFIG_KERNEL_PRIORITY
    #define CONFIG_KERNEL_PRIORITY      (2)
#endif
#ifndef CONFIG_USING_CONSOLE
    #define CONFIG_USING_CONSOLE        (1)
#endif
#ifndef CONFIG_USING_FINSH
    #define CONFIG_USING_FINSH          (CONFIG_USING_CONSOLE)
#endif
#if (CONFIG_USING_CONSOLE)
    #ifndef CONFIG_SERIAL_DEVICE
        #define CONFIG_SERIAL_DEVICE    (Serial)
    #endif
#endif

/* Debug Options */
// #define RT_DEBUG
// #define RT_USING_OVERFLOW_CHECK
// #define RT_DEBUG_INIT                   (1)
// #define RT_DEBUG_MEM                    (1)
// #define RT_DEBUG_SCHEDULER              (1)
// #define RT_DEBUG_IPC                    (1)
// #define RT_DEBUG_TIMER                  (1)

/* System Options */
#define RT_NAME_MAX                     (16)
#define RT_ALIGN_SIZE                   (4)
#define RT_THREAD_PRIORITY_MAX          (32)
#define RT_TICK_PER_SECOND              (100)

/* Timer Options */
// #define RT_USING_TIMER_SOFT
// #define RT_TIMER_THREAD_PRIO            (4)
// #define RT_TIMER_THREAD_STACK_SIZE      (512)

/* Utility Options */
#define RT_USING_DEVICE                 /* Required by IPC */
#define RT_USING_SEMAPHORE              /* Required by ? */
#define RT_USING_MUTEX                  /* Required by ? */
#define RT_USING_EVENT                  /* Required by ? */
#define RT_USING_MAILBOX                /* Required by ? */
#define RT_USING_MESSAGEQUEUE           /* Required by ? */
// #define RT_USING_HOOK
// #define RT_USING_COMPONENTS_INIT

/* Memory Management Options */
#define RT_USING_MEMPOOL
// #define RT_USING_MEMHEAP
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM

/* Console Options */
#if (CONFIG_USING_CONSOLE)
    #define RT_USING_CONSOLE
    #define RT_CONSOLEBUF_SIZE          (128)
#endif

/* FinSH Options */
#if (CONFIG_USING_FINSH)
    #define RT_USING_FINSH
    // #define FINSH_USING_SYMTAB          /* Not supported as no access to linker script */
    #define FINSH_USING_DESCRIPTION     /* Show command description with list() */
    #define FINSH_USING_HISTORY
    #define FINSH_THREAD_PRIORITY       (20)
    #define FINSH_THREAD_STACK_SIZE     (2 * 1024)
    // #define FINSH_USING_MSH
#endif

/* Other Options */
// #define RT_USING_COMPONENTS_INIT        /* Not supported as no access to linker script */

#endif /* __RTTHREAD_CFG_H__ */
