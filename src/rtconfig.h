/***************************************************************************//**
 * @file    rtconfig.h
 * @brief   Arduino RT-Thread library config
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __RTCONFIG_H__
#define __RTCONFIG_H__

/* Porting Options */
#define CONFIG_ARDUINO

#define CONFIG_TICK_PER_SECOND          (1000)  /* Arduino */

#ifndef CONFIG_HEAP_SIZE
#define CONFIG_HEAP_SIZE                (20 * 1024)
#endif

#ifndef CONFIG_PRIORITY_MAX
#define CONFIG_PRIORITY_MAX             (3)
#endif

#ifndef CONFIG_KERNEL_PRIORITY
#define CONFIG_KERNEL_PRIORITY          (2)
#endif

#ifndef CONFIG_USING_CONSOLE
#define CONFIG_USING_CONSOLE            (1)
#endif

#ifndef CONFIG_USING_FINSH
#define CONFIG_USING_FINSH              (CONFIG_USING_CONSOLE)
#endif

#ifndef CONFIG_USING_SPISD
#define CONFIG_USING_SPISD              (ARDUINO_ARCH_SAMD)
#endif

#if (CONFIG_USING_CONSOLE)
#ifndef CONFIG_SERIAL_DEVICE
#define CONFIG_SERIAL_DEVICE            (Serial)
#endif
#endif /* CONFIG_USING_CONSOLE */

#if (CONFIG_USING_SPISD)
#define CONFIG_USING_SPI1               (1)
#ifndef CONFIG_SD_CS_PIN
#ifdef ARDUINO_SAMD_MKRZERO
#define CONFIG_SD_CS_PIN                (SDCARD_SS_PIN)
#else
#error "Please define CONFIG_SD_CS_PIN!"
#endif /* ARDUINO_SAMD_MKRZERO */
#endif /* CONFIG_SD_CS_PIN */

#ifndef CONFIG_SD_SPI_CHANNEL
#ifdef ARDUINO_SAMD_MKRZERO
#define CONFIG_SD_SPI_CHANNEL           1   /* (1) is wrong -_-! */
#else
#error "Please define CONFIG_SD_SPI_CHANNEL!"
#endif /* ARDUINO_SAMD_MKRZERO */
#endif /* CONFIG_SD_SPI_CHANNEL */
#endif /* CONFIG_USING_SPISD */

#ifndef CONFIG_USING_SPI0
#define CONFIG_USING_SPI0               (0)
#endif

#ifndef CONFIG_USING_SPI1
#define CONFIG_USING_SPI1               (0)
#endif

/* Debug Options */
// #define RT_DEBUG
// #define RT_USING_OVERFLOW_CHECK
// #define RT_DEBUG_INIT                   (1)
// #define RT_DEBUG_MEM                    (1)
// #define RT_DEBUG_SCHEDULER              (1)
// #define RT_DEBUG_IPC                    (1)
// #define RT_DEBUG_TIMER                  (1)
// #define RT_DEBUG_THREAD                 (1)

/* System Options */
#define RT_NAME_MAX                     (16)
#define RT_ALIGN_SIZE                   (4)
#define RT_THREAD_PRIORITY_MAX          (32)
#define RT_TICK_PER_SECOND              (100)

/* Arduino Thread Options */
#ifndef CONFIG_ARDUINO_STACK_SIZE
#define CONFIG_ARDUINO_STACK_SIZE       (4 * 1024)
#endif
#ifndef CONFIG_ARDUINO_PRIORITY
#define CONFIG_ARDUINO_PRIORITY         (RT_THREAD_PRIORITY_MAX >> 1)
#endif
#ifndef CONFIG_ARDUINO_TICK
#define CONFIG_ARDUINO_TICK             (16)
#endif

/* Timer Options */
// #define RT_USING_TIMER_SOFT
// #define RT_TIMER_THREAD_PRIO            (4)
// #define RT_TIMER_THREAD_STACK_SIZE      (512)

/* Utility Options */
#define RT_USING_DEVICE                 /* Required by IPC, DRV */
#define RT_USING_SEMAPHORE              /* Required by FINSH */
#define RT_USING_MUTEX                  /* Required by DFS, DRV */
#define RT_USING_EVENT                  /* Required by ? */
#define RT_USING_MAILBOX                /* Required by ? */
#define RT_USING_MESSAGEQUEUE           /* Required by ? */
// #define RT_USING_SIGNALS                /* Required by ? */
// #define RT_USING_HOOK
// #define RT_USING_IDLE_HOOK

/* Memory Management Options */
#define RT_USING_MEMPOOL                /* Required by SIG */
// #define RT_USING_MEMHEAP
#define RT_USING_HEAP
#define RT_USING_SMALL_MEM

/* Console Options */
#if (CONFIG_USING_CONSOLE)
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE              (128)
#endif /* CONFIG_USING_CONSOLE */

/* FinSH Options */
#if (CONFIG_USING_FINSH)
#define RT_USING_FINSH
// #define FINSH_USING_SYMTAB           /* Not supported as no access to linker script */
#define FINSH_USING_DESCRIPTION
#define FINSH_USING_HISTORY
#define FINSH_THREAD_PRIORITY           (20)
#define FINSH_THREAD_STACK_SIZE         (4 * 1024)
// #define FINSH_USING_MSH
#endif /* CONFIG_USING_FINSH */

/* DFS Options */
#if (CONFIG_USING_SPISD)
#define RT_USING_DFS
// #define RT_USING_DFS_MNTTABLE
#define RT_USING_DFS_ELMFAT
#define DFS_USING_WORKDIR
#define RT_DFS_ELM_CODE_PAGE            (437)
// #define RT_DFS_ELM_USE_EXFAT
#ifdef RT_DFS_ELM_USE_EXFAT
#define RT_DFS_ELM_USE_LFN              (2)
#define RT_DFS_ELM_MAX_LFN              (255)
#endif /* RT_DFS_ELM_USE_EXFAT */
#endif /* CONFIG_USING_SPISD */

/* Other Options */
// #define RT_USING_COMPONENTS_INIT     /* Not supported as no access to linker script */

#endif /* __RTCONFIG_H__ */
