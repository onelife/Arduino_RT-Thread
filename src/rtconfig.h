/***************************************************************************//**
 * @file    rtconfig.h
 * @brief   Arduino RT-Thread library config
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __RTCONFIG_H__
#define __RTCONFIG_H__

/* User Config */

// #define CONFIG_USING_ADAFRUIT_TFT_CAPACITIVE
// #define CONFIG_USING_TINYSCREEN
// #define CONFIG_USING_SSD1306_SPI4

/* Hardware Config */

/* Longan Nano */
#ifdef BOARD_SIPEED_LONGAN_NANO
# define ARDUINO_ARCH_RISCV
# define CONFIG_USING_DRIVER_SERIAL     (1)
# define CONFIG_USING_UART0             (1)
# define CONFIG_USING_DRIVER_SPI        (1)

# define CONFIG_USING_SPI1              (1)
# define CONFIG_USING_SPISD             (1)
# define CONFIG_SD_SPI_CHANNEL          1

# define CONFIG_USING_SPI0              (1)
# define CONFIG_USING_ST7735            (1)
# define CONFIG_ST7735_PORT             (GPIOB)
# define CONFIG_ST7735_PORT_CLK         (RCU_GPIOB)
# define CONFIG_ST7735_CS_PIN           (GPIO_PIN_2)
# define CONFIG_ST7735_DC_PIN           (GPIO_PIN_0)
# define CONFIG_ST7735_RST_PIN          (GPIO_PIN_1)
# define CONFIG_ST7735_SPI_CHANNEL      0

# define CONFIG_USING_GUI               (1)
# define CONFIG_GUI_WIDTH               (160)
# define CONFIG_GUI_HIGH                (80)

# define CONFIG_HEAP_SIZE               (20 * 1024)
# define IDLE_THREAD_STACK_SIZE         (1 * 512)
#endif /* BOARD_SIPEED_LONGAN_NANO */

/* Adafruit 2.8" TFT Touch Shield v2 (Capacitive) */
#ifdef CONFIG_USING_ADAFRUIT_TFT_CAPACITIVE
# ifdef ARDUINO_SAM_DUE
#  define CONFIG_USING_SPI0             (1)
#  define CONFIG_USING_IIC1             (1)

#  define CONFIG_USING_MODULE           (1)
#  define CONFIG_USING_SPISD            (1)
#  define CONFIG_SD_CS_PIN              (4)
#  define CONFIG_SD_SPI_CHANNEL         0

#  define CONFIG_USING_ILI              (1)
#  define CONFIG_ILI_CS_PIN             (10)
#  define CONFIG_ILI_DC_PIN             (9)
#  define CONFIG_ILI_SPI_CHANNEL        0

#  define CONFIG_USING_FT6206           (1)
#  define CONFIG_FT6206_INT_PIN         (7)
#  define CONFIG_FT6206_IIC_CHANNEL     1

#  define CONFIG_USING_GUI              (1)
#  define CONFIG_GUI_WIDTH              (240)
#  define CONFIG_GUI_HIGH               (320)
# else
#  error "Not supporting yet!"
# endif /* ARDUINO_SAM_DUE */
#endif /* CONFIG_USING_ADAFRUIT_TFT_CAPACITIVE */

/* Pocket Arcade */
#ifdef CONFIG_USING_TINYSCREEN
# define CONFIG_SERIAL_DEVICE           (SerialUSB)
# define CONFIG_USING_SPI0              (1)
# define CONFIG_USING_SPI1              (1)

// # define CONFIG_USING_MODULE            (1)
# define CONFIG_USING_SPISD             (1)
# define CONFIG_SD_CS_PIN               (SS)
# define CONFIG_SD_SPI_CHANNEL          0

# define CONFIG_USING_SSD1331           (1)
# define CONFIG_SSD_CS_PIN              (38)
# define CONFIG_SSD_DC_PIN              (22)
# define CONFIG_SSD_RST_PIN             (26)
# define CONFIG_SSD_PWR_PIN             (27)
# define CONFIG_SSD_SPI_CHANNEL         1

# define CONFIG_USING_BUTTON            (6)     /* UP, DOWN, LEFT, RIGHT, A, B */
# define CONFIG_BUTTON_PIN              { 42, 19, 25, 15, 45, 44 }
# define CONFIG_BUTTON_CODE             { 273, 274, 276, 275, 97, 98 }

# define CONFIG_USING_GUI               (1)
# define CONFIG_GUI_WIDTH               (96)
# define CONFIG_GUI_HIGH                (64)
#endif /* CONFIG_USING_TINYSCREEN */

/* Arduino MKR Zero */
#ifdef ARDUINO_SAMD_MKRZERO
# define CONFIG_USING_SPI1              (1)

# define CONFIG_USING_SPISD             (1)
# define CONFIG_SD_CS_PIN               (SDCARD_SS_PIN)
# define CONFIG_SD_SPI_CHANNEL          1       /* (1) is wrong -_-! */
#endif /* ARDUINO_SAMD_MKRZERO */

/* SSD1306 */
#ifdef CONFIG_USING_SSD1306_SPI4
# define CONFIG_USING_SPI0              (1)

# define CONFIG_USING_SSD1306           (1)
# define CONFIG_SSD_CS_PIN              (3)
# define CONFIG_SSD_DC_PIN              (4)
# define CONFIG_SSD_RST_PIN             (5)
# define CONFIG_SSD_SPI_CHANNEL         0

# define CONFIG_USING_GUI               (1)
# define CONFIG_GUI_WIDTH               (128)
# define CONFIG_GUI_HIGH                (64)
#endif /* CONFIG_USING_SSD1306_SPI4 */

/* STM32duino */
#ifdef ARDUINO_ARCH_STM32
# define CONFIG_NO_ERRNO
# define RT_TICK_PER_SECOND             (1000)
# define CONFIG_HEAP_SIZE               (60 * 1024)
# define CONFIG_ARDUINO_STACK_SIZE      (12 * 512)
# define CONFIG_USING_DRIVER_RTC        (1)     /* Require STM32duino_RTC library */
#endif /* ARDUINO_ARCH_STM32 */


/* Arduino Config */

#define CONFIG_ARDUINO
#define CONFIG_TICK_PER_SECOND          (1000)  /* Platform */

#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
# ifndef CONFIG_PRIORITY_MAX
#  define CONFIG_PRIORITY_MAX           (3)     /* NVIC */
# endif
# ifndef CONFIG_KERNEL_PRIORITY
#  define CONFIG_KERNEL_PRIORITY        (2)     /* Platform */
# endif
#endif /* defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32) */

#if defined(ARDUINO_ARCH_SAM)
# define SERIAL_IRQn                    (UART_IRQn)
#elif defined(ARDUINO_ARCH_SAMD)
# define SERIAL_IRQn                    (USB_IRQn)
#elif defined(ARDUINO_ARCH_STM32)
# if defined(ARDUINO_NUCLEO_F767ZI)
#  define SERIAL_IRQn                   (USART3_IRQn)
# endif
#elif defined(ARDUINO_ARCH_RISCV)
 /* none */
#else
# warning "Unsupported architecture!"
#endif

#ifndef CONFIG_HEAP_SIZE
#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_STM32)
#  define CONFIG_HEAP_SIZE              (40 * 1024)
# else
#  define CONFIG_HEAP_SIZE              (20 * 1024)
# endif
#endif


/* Default User Config */

#ifndef CONFIG_USART_SPEED
# define CONFIG_USART_SPEED             (115200)
#endif

#ifndef CONFIG_USING_DRIVER_SERIAL
# define CONFIG_USING_DRIVER_SERIAL     (0)
#endif

#ifndef CONFIG_USING_UART0
# define CONFIG_USING_UART0             (0)
#endif
#ifndef CONFIG_USING_UART1
# define CONFIG_USING_UART1             (0)
#endif

#ifndef CONFIG_USING_DRIVER_SPI
# define CONFIG_USING_DRIVER_SPI        (0)
#endif

#ifndef CONFIG_USING_SPI0
# define CONFIG_USING_SPI0              (0)
#endif
#ifndef CONFIG_USING_SPI1
# define CONFIG_USING_SPI1              (0)
#endif

#if CONFIG_USING_DRIVER_SERIAL
# if !CONFIG_USING_UART0 && !CONFIG_USING_UART1
#  error "CONFIG_USING_UARTx must be enabled for Serial Driver"
# endif
#endif

#if CONFIG_USING_DRIVER_SPI
# if !CONFIG_USING_SPI0 && !CONFIG_USING_SPI1
#  error "CONFIG_USING_SPIx must be enabled for SPI Driver"
# endif
#endif

#ifndef CONFIG_USING_DRIVER_RTC
# define CONFIG_USING_DRIVER_RTC        (0)
#endif

#ifndef CONFIG_USING_CONSOLE
# define CONFIG_USING_CONSOLE           (1)
#endif

#ifndef CONFIG_USING_FINSH
# define CONFIG_USING_FINSH             (CONFIG_USING_CONSOLE)
#endif

#ifndef CONFIG_USING_MSH
# define CONFIG_USING_MSH               (1)
#endif

#ifndef CONFIG_USING_LOG
# define CONFIG_USING_LOG               (1)
#endif

#ifndef CONFIG_USING_MODULE
# define CONFIG_USING_MODULE            (0)
#endif

#ifndef CONFIG_USING_GUI
# define CONFIG_USING_GUI               (0)
#endif

#ifndef CONFIG_USING_BUTTON
# define CONFIG_USING_BUTTON            (0)
#endif

#ifndef CONFIG_USING_SPISD
# define CONFIG_USING_SPISD             (0)
#endif

#ifndef CONFIG_USING_EXFAT
# define CONFIG_USING_EXFAT             (0)
#endif

#ifndef CONFIG_USING_ILI
# define CONFIG_USING_ILI               (0)
#endif

#ifndef CONFIG_USING_SSD1331
# define CONFIG_USING_SSD1331           (0)
#endif

#ifndef CONFIG_USING_FT6206
# define CONFIG_USING_FT6206            (0)
#endif

#ifndef CONFIG_USING_ST7735
# define CONFIG_USING_ST7735            (0)
#endif

#ifndef CONFIG_USING_SPI0
# define CONFIG_USING_SPI0              (0)
#endif

#ifndef CONFIG_USING_SPI1
# define CONFIG_USING_SPI1              (0)
#endif

#ifndef CONFIG_USING_IIC0
# define CONFIG_USING_IIC0              (0)
#endif

#ifndef CONFIG_USING_IIC1
# define CONFIG_USING_IIC1              (0)
#endif


/* Config Check */

#if (CONFIG_USING_CONSOLE)
# ifndef CONFIG_SERIAL_DEVICE
#  define CONFIG_SERIAL_DEVICE          (Serial)
# endif
#endif /* CONFIG_USING_CONSOLE */

#if (CONFIG_USING_LOG)
# define RT_USING_ULOG
# define ULOG_OUTPUT_LVL                (LOG_LVL_DBG) // (LOG_LVL_INFO)
# define ULOG_ASSERT_ENABLE
# define ULOG_USING_COLOR
// # define ULOG_USING_ISR_LOG
// # define ULOG_USING_SYSLOG
// # define ULOG_USING_FILTER
# define ULOG_OUTPUT_TIME
// # define ULOG_TIME_USING_TIMESTAMP
# define ULOG_OUTPUT_LEVEL
# define ULOG_OUTPUT_TAG
# define ULOG_OUTPUT_THREAD_NAME
# if (CONFIG_USING_CONSOLE)
#  define ULOG_BACKEND_USING_CONSOLE
# endif 
#endif /* CONFIG_USING_LOG */

#if (CONFIG_USING_MODULE)
# define RT_USING_MODULE
# define MODULE_THREAD_PRIORITY         (RT_THREAD_PRIORITY_MAX - 1)
# define MODULE_THREAD_STACK_SIZE       (4 * 512)
# define IDLE_THREAD_STACK_SIZE         (1 * 512)
#endif /* CONFIG_USING_MODULE */

#if (CONFIG_USING_SPISD)
# if (!defined(CONFIG_USING_SPI0) && !defined(CONFIG_USING_SPI1))
#  error "Please define CONFIG_USING_SPIx!"
# endif
# ifndef CONFIG_SD_SPI_CHANNEL
#  error "Please define CONFIG_SD_SPI_CHANNEL!"
# endif
# if (!CONFIG_USING_DRIVER_SPI)
#  ifndef CONFIG_SD_CS_PIN
#   error "Please define CONFIG_SD_CS_PIN!"
#  endif
# else /* !CONFIG_USING_DRIVER_SPI */
#  define RT_USING_SPI_MSD
# endif /* !CONFIG_USING_DRIVER_SPI */
#endif /* CONFIG_USING_SPISD */

#if (CONFIG_USING_ILI)
# ifndef CONFIG_ILI_CS_PIN
#  error "Please define CONFIG_ILI_CS_PIN"
# endif
# ifndef CONFIG_ILI_DC_PIN
#  error "Please define CONFIG_ILI_DC_PIN"
# endif
# ifndef CONFIG_ILI_SPI_CHANNEL
#  error "Please define CONFIG_ILI_SPI_CHANNEL"
# endif
#endif /* CONFIG_USING_ILI */

#if (CONFIG_USING_FT6206)
# ifndef CONFIG_FT6206_IIC_CHANNEL
#  error "Please define CONFIG_FT6206_IIC_CHANNEL"
# endif
#endif /* CONFIG_USING_FT6206 */

#if (CONFIG_USING_GUI)
# ifndef CONFIG_GUI_WIDTH
#  error "Please define CONFIG_GUI_WIDTH"
# endif
# ifndef CONFIG_GUI_HIGH
#  error "Please define CONFIG_GUI_HIGH"
# endif
#endif /* CONFIG_USING_GUI */


/* Debug Config */

// #define RT_DEBUG
// #define RT_USING_OVERFLOW_CHECK
// #define RT_USING_MEMTRACE
// #define RT_DEBUG_INIT                   (1)
// #define RT_DEBUG_MEM                    (1)
// #define RT_DEBUG_SCHEDULER              (1)
// #define RT_DEBUG_IPC                    (1)
// #define RT_DEBUG_TIMER                  (1)
// #define RT_DEBUG_THREAD                 (1)



/* RT-Thread Kernel */

#define RT_NAME_MAX                     (16)
#define RT_ALIGN_SIZE                   (4)
#define RT_THREAD_PRIORITY_MAX          (32)
#ifndef RT_TICK_PER_SECOND
# define RT_TICK_PER_SECOND             (100)
#endif
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
// #define RT_USING_TIMER_SOFT
// #define RT_TIMER_THREAD_PRIO 4
// #define RT_TIMER_THREAD_STACK_SIZE 512

/* kservice optimization */

#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE              /* Required by FINSH, CMSIS-OS */
#define RT_USING_MUTEX                  /* Required by DFS, DRV, CMSIS-OS */
// #define RT_USING_EVENT                  /* Required by CMSIS-OS */
#define RT_USING_MAILBOX                /* Required by GUI */
// #define RT_USING_MESSAGEQUEUE           /* Required by CMSIS-OS */
// #define RT_USING_SIGNALS                /* Required by ? */

/* Memory Management */

#define RT_USING_MEMPOOL                /* Required by SIG, GUI, CMSIS-OS */
#define RT_USING_SMALL_MEM
// #define RT_USING_MEMHEAP
// #define RT_MEMHEAP_FAST_MODE
#define RT_USING_SMALL_MEM_AS_HEAP
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE                 /* Required by IPC, DRV */
#if (CONFIG_USING_CONSOLE)
# define RT_USING_CONSOLE
# define RT_CONSOLEBUF_SIZE             (128)
#endif /* CONFIG_USING_CONSOLE */
// #define RT_CONSOLE_DEVICE_NAME "uart"
// #define RT_VER_NUM 0x40100

/* RT-Thread Components */

// #define RT_USING_USER_MAIN
// #define RT_MAIN_THREAD_STACK_SIZE 2048
// #define RT_MAIN_THREAD_PRIORITY 10
#if (CONFIG_USING_FINSH)
#define RT_USING_FINSH
# if (CONFIG_USING_MSH)
#  define RT_USING_MSH
#  define FINSH_USING_MSH
# endif
#define FINSH_THREAD_PRIORITY           ((RT_THREAD_PRIORITY_MAX >> 1) + (RT_THREAD_PRIORITY_MAX >> 3))
#define FINSH_THREAD_STACK_SIZE         (4 * 512)
#define FINSH_USING_HISTORY
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#endif /* CONFIG_USING_FINSH */



/* Arduino Thread Config */

#ifndef CONFIG_ARDUINO_STACK_SIZE
# define CONFIG_ARDUINO_STACK_SIZE      (4 * 512)
#endif
#ifndef CONFIG_ARDUINO_PRIORITY
# define CONFIG_ARDUINO_PRIORITY        (RT_THREAD_PRIORITY_MAX >> 1)
#endif
#ifndef CONFIG_ARDUINO_TICK
# define CONFIG_ARDUINO_TICK            (RT_TICK_PER_SECOND / 10)
#endif








/* RTT Driver COnfig */

#if CONFIG_USING_DRIVER_SERIAL || CONFIG_USING_DRIVER_SPI
# define CONFIG_USING_BSP               (1)
#else
# define CONFIG_USING_BSP               (0)
#endif
#if CONFIG_USING_DRIVER_SERIAL
# define RT_USING_DEVICE_IPC
#endif
#if CONFIG_USING_DRIVER_SERIAL
# define RT_USING_SERIAL
#endif
#if CONFIG_USING_DRIVER_SPI
# define RT_USING_SPI
#endif
#if CONFIG_USING_DRIVER_RTC
# define RT_USING_RTC
# define RT_USING_ALARM
#endif




/* File System Config */

#if (CONFIG_USING_SPISD)
# define RT_USING_DFS
# define DFS_USING_POSIX
// # define RT_USING_DFS_MNTTABLE          /* Mount table */
# define RT_USING_DFS_ELMFAT
# define DFS_USING_WORKDIR
# define DFS_FILESYSTEMS_MAX            (1)     /* Max number of fs */
# define DFS_FD_MAX                     (4)     /* Max number of open file */
# define RT_DFS_ELM_CODE_PAGE           437     /* (xxx) is wrong -_-! */
# if (CONFIG_USING_EXFAT)
#  define RT_DFS_ELM_USE_EXFAT
# endif
# if defined(RT_DFS_ELM_USE_EXFAT) || (RT_DFS_ELM_CODE_PAGE >= 900)
#  define RT_DFS_ELM_USE_LFN            (2)
#  define RT_DFS_ELM_MAX_LFN            (255)
// #  define RT_DFS_ELM_LFN_UNICODE
# endif /* defined(RT_DFS_ELM_USE_EXFAT) || (RT_DFS_ELM_CODE_PAGE >= 900) */
#endif /* CONFIG_USING_SPISD */


/* Unsupported Config */

#ifdef FINSH_USING_SYMTAB
# undef FINSH_USING_SYMTAB              /* Reason: no access to linker script */
#endif
#ifdef RT_USING_COMPONENTS_INIT
# undef RT_USING_COMPONENTS_INIT        /* Reason: no access to linker script */
#endif


/* User provided config */

#if defined __has_include
# if __has_include("rtconfig_extra.h")
#  include "rtconfig_extra.h"
# endif
#endif

#endif /* __RTCONFIG_H__ */
