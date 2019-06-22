/***************************************************************************//**
 * @file    rtt.cpp
 * @brief   Arduino RT-Thread library main
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include <Arduino.h>
#include "rtt.h"

#if (CONFIG_PRIORITY_MAX > ((1 << __NVIC_PRIO_BITS) - 1))
    #error "CONFIG_PRIORITY_MAX is more than max hardware priority level"
#endif

#if (CONFIG_KERNEL_PRIORITY > CONFIG_PRIORITY_MAX)
    #error "CONFIG_KERNEL_PRIORITY can not be more than CONFIG_PRIORITY_MAX"
#endif

#if (RT_TICK_PER_SECOND > CONFIG_TICK_PER_SECOND)
    #error "RT_TICK_PER_SECOND can not be more than CONFIG_TICK_PER_SECOND"
#endif


#define KERNEL_PRIORITY     ((CONFIG_KERNEL_PRIORITY << (8 - __NVIC_PRIO_BITS)) & 0xff)
#define TICK_COUNT          (CONFIG_TICK_PER_SECOND / RT_TICK_PER_SECOND)
#if (CONFIG_USING_CONSOLE)
    #define SERIAL_DEVICE   CONFIG_SERIAL_DEVICE
#endif
#if (CONFIG_USING_FINSH)
    #define SERIAL_NAME     "Serial"
#endif


#if CONFIG_USING_FINSH

/* === Map Arduino "Serial" to RT-Thread Device === */

static struct rt_device serial_dev;

static rt_err_t serial_close(rt_device_t dev) {
    (void)dev;
    SERIAL_DEVICE.end();
    return RT_EOK;
}

static rt_size_t serial_read(rt_device_t dev, rt_off_t pos,
    void *buffer, rt_size_t size) {
    (void)dev;
    (void)pos;
    return SERIAL_DEVICE.readBytes((char *)buffer, size);
}

static rt_size_t serial_write(rt_device_t dev, rt_off_t pos,
    const void *buffer, rt_size_t size) {
    (void)dev;
    (void)pos;
    return SERIAL_DEVICE.write((const uint8_t *)buffer, size);
}

static rt_err_t arduino_serial_init(void) {
    rt_uint32_t flag = RT_DEVICE_FLAG_RDWR | \
                       RT_DEVICE_FLAG_STREAM | \
                       RT_DEVICE_FLAG_INT_RX;

    serial_dev.type         = RT_Device_Class_Char;
    serial_dev.rx_indicate  = RT_NULL;
    serial_dev.tx_complete  = RT_NULL;
    serial_dev.init         = RT_NULL;
    serial_dev.open         = RT_NULL;
    serial_dev.close        = serial_close;
    serial_dev.read         = serial_read;
    serial_dev.write        = serial_write;
    serial_dev.control      = RT_NULL;
    // serial_dev.user_data    = SERIAL_DEVICE;

    return rt_device_register(&serial_dev, SERIAL_NAME, flag);
}

#endif /* CONFIG_USING_FINSH */


extern "C" {
    /* === Override RT-Thread Functions === */

#if defined(RT_DEBUG)
    void assert_failed(uint8_t * file, uint32_t line) {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);
        rt_kprintf("\r\nWrong parameter value detected on\r\n");
        rt_kprintf("\tfile  %s\r\n", file);
        rt_kprintf("\tline  %d\r\n", line);

        while (1) {
        }
    }
#endif /* defined(RT_DEBUG) */

    rt_base_t rt_hw_interrupt_disable(void) {
        rt_base_t original_level;

        #if defined(ARDUINO_ARCH_SAM)
            original_level = __get_BASEPRI();
            // not disable interrupt but raise base priority
            __set_BASEPRI(KERNEL_PRIORITY);
        #elif defined(ARDUINO_ARCH_SAMD)
            original_level = __get_PRIMASK();
            __set_PRIMASK(1);
        #endif

        return original_level;
    }

    void rt_hw_interrupt_enable(rt_base_t level) {
        (void)level;

        #if defined(ARDUINO_ARCH_SAM)
            // reset base priority
            __set_BASEPRI(0);
        #elif defined(ARDUINO_ARCH_SAMD)
            __set_PRIMASK(0);
        #endif
    }

    void rt_hw_console_output(const char *str) {
        #if (CONFIG_USING_CONSOLE)
            if (NULL != str) {
                SERIAL_DEVICE.print(str);
            }
        #endif
    }

    int sysTickHook(void) {
        static unsigned int cnt = 0;
        cnt++;
        if (cnt >= TICK_COUNT) {
            cnt = 0;
            rt_interrupt_enter();
            rt_tick_increase();
            rt_interrupt_leave();
        }

        #if CONFIG_USING_FINSH
            if ((serial_dev.rx_indicate != RT_NULL) && \
                 SERIAL_DEVICE.available()) {
                serial_dev.rx_indicate(&serial_dev, SERIAL_DEVICE.available());
            }
        #endif
        return 0;
    }

} /* extern "C" */


/* === Initialize Conponents and Handle Arduino Stuff === */

extern "C" {
    #if CONFIG_USING_SPI0 || CONFIG_USING_SPI1
    # include "components/arduino/drv_spi.h"
    #endif
    #if CONFIG_USING_IIC0 || CONFIG_USING_IIC1
    # include "components/arduino/drv_iic.h"
    #endif
    #if CONFIG_USING_FINSH
    # include "components/finsh/shell.h"
    #endif
    #ifdef RT_USING_DFS
    # include "components/dfs/include/dfs.h"
    # include "components/dfs/include/dfs_fs.h"
    # include "components/dfs/filesystems/elmfat/dfs_elm.h"
    #endif
    #ifdef RT_USING_ULOG
    # define LOG_TAG "RTT"
    # include "components/utilities/ulog/ulog.h"
    #else
    # define LOG_E(format, args...) rt_kprintf(format "\n", ##args)
    # define LOG_I(format, args...) rt_kprintf(format "\n", ##args)
    #endif
    #ifdef ULOG_BACKEND_USING_CONSOLE
    extern int ulog_console_backend_init(void);
    #endif
    #ifdef RT_USING_MODULE
    # include "components/libc/libdl/dlmodule.h"
    #endif
    #if CONFIG_USING_SPISD
    # include "components/arduino/drv_spisd.h"
    #endif
    #if CONFIG_USING_ILI
    # include "components/arduino/drv_spiili.h"
    #endif
    #if CONFIG_USING_FT6206
    # include "components/arduino/drv_iic_ft6206.h"
    #endif
}
#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

/* Driver init */
void rt_driver_init(void) {
    rt_err_t ret;

    #if (CONFIG_USING_CONSOLE)
        SERIAL_DEVICE.begin(CONFIG_USART_SPEED);
        while (!SERIAL_DEVICE);
    #endif
    #if (CONFIG_USING_SPI0 || CONFIG_USING_SPI1)
        ret = bsp_hw_spi_init();
        RT_ASSERT(RT_EOK == ret);
    #endif
    #if (CONFIG_USING_IIC0 || CONFIG_USING_IIC1)
        ret = bsp_hw_iic_init();
        RT_ASSERT(RT_EOK == ret);
    #endif
    (void)ret;
}

/* High level driver init */
void rt_high_driver_init(void) {
    rt_err_t ret;

    #if CONFIG_USING_SPISD
        ret = bsp_hw_spiSd_init();
        RT_ASSERT(RT_EOK == ret);
    #endif
    #if CONFIG_USING_ILI
        ret = bsp_hw_spiIli_init();
        RT_ASSERT(RT_EOK == ret);
    #endif
    #if CONFIG_USING_FT6206
        ret = bsp_hw_ft6206_init();
        RT_ASSERT(RT_EOK == ret);
    #endif
    (void)ret;

    #if CONFIG_USING_GUI
        RTT_GUI.begin();
    #endif
}

/* Component init */
void rt_components_init(void) {
    #ifdef ULOG_BACKEND_USING_CONSOLE
        (void)ulog_console_backend_init();
    #elif defined(RT_USING_ULOG)
        (void)ulog_init();
    #endif

    /* INIT_BOARD_EXPORT */
    rt_high_driver_init();

    /* INIT_PREV_EXPORT */
    #ifdef RT_USING_DFS
        (void)dfs_init();
    #endif

    /* INIT_DEVICE_EXPORT */

    /* INIT_COMPONENT_EXPORT */
    #ifdef RT_USING_DFS_ELMFAT
        (void)elm_init();
    #endif
    #ifdef RT_USING_MODULE
        // (void)rt_system_dlmodule_init();
    #endif

    /* INIT_ENV_EXPORT */
    #ifdef RT_USING_DFS_MNTTABLE
        // dfs_mount_table
    #elif defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
        if (dfs_mount(SD_NAME, "/", "elm", 0, 0)) {
            LOG_E("[E] Mount " SD_NAME " failed!");
        } else {
            LOG_I("Mount " SD_NAME " to \"/\"");
        }
    #endif

    /* INIT_APP_EXPORT */
    #if CONFIG_USING_FINSH
        arduino_serial_init();
        rt_console_set_device(SERIAL_NAME);
        finsh_system_init();
    #endif
}

/* Arduino thread */
void arduino_thread_entry(void *param) {
    rt_thread_t self = rt_thread_self();
    (void)param;

    /* init components */
    rt_components_init();

    /* reset priority */
    rt_thread_control(self, RT_THREAD_CTRL_CHANGE_PRIORITY,
        &self->init_priority);

    /* run Arduino loop here */
    while (1) {
        loop();
        if (serialEventRun) serialEventRun();
        rt_thread_sleep(1);
    }
}

static rt_uint8_t arduino_stack[CONFIG_ARDUINO_STACK_SIZE];
static struct rt_thread arduino_thread;

/* Application init */
void rt_application_init(void) {
    rt_uint32_t tmp_prio = 1;
    rt_err_t ret = rt_thread_init(
        &arduino_thread, "Arduino",
        arduino_thread_entry, RT_NULL,
        arduino_stack, sizeof(arduino_stack),
        CONFIG_ARDUINO_PRIORITY, CONFIG_ARDUINO_TICK);
    RT_ASSERT(RT_EOK == ret);

    /* raise priority */
    ret = rt_thread_startup(&arduino_thread);
    RT_ASSERT(RT_EOK == ret);
    ret = rt_thread_control(&arduino_thread, RT_THREAD_CTRL_CHANGE_PRIORITY,
        &tmp_prio);
    RT_ASSERT(RT_EOK == ret);

    (void)ret;
}


/* === User Override Functions === */

void rt_setup(void) __attribute__((weak));
void loop(void) __attribute__((weak));

void rt_setup(void) { }
void loop(void) { }

/* === RT-Thread Class === */

void RT_Thread::begin(void) {
    #if defined(ARDUINO_SAM_DUE)
        NVIC_SetPriority(UART_IRQn, CONFIG_PRIORITY_MAX);
    #elif defined(ARDUINO_SAMD_MKRZERO)
        NVIC_SetPriority(USB_IRQn, CONFIG_PRIORITY_MAX);
    #else
        #warning "Unsupported board!"
    #endif
    NVIC_SetPriority(SysTick_IRQn, CONFIG_KERNEL_PRIORITY);
    NVIC_SetPriority(PendSV_IRQn, CONFIG_KERNEL_PRIORITY);

    /* disable interrupt*/
    rt_hw_interrupt_disable();

    #ifdef RT_USING_HEAP
        /* init heap */
        rt_system_heap_init((void *)&rtt_heap,
            (void *)&rtt_heap[CONFIG_HEAP_SIZE-1]);
    #endif

    /* init tick */
    rt_system_tick_init();

    /* init kernel object */
    rt_system_object_init();

    /* init timer */
    rt_system_timer_init();

    /* init scheduler */
    rt_system_scheduler_init();

    #ifdef RT_USING_SIGNALS
        /* init signal */
        // rt_system_signal_init();
    #endif

    /* init driver */
    rt_driver_init();

    /* show version */
    rt_show_version();

    /* init timer thread */
    rt_system_timer_thread_init();

    /* init idle thread */
    rt_thread_idle_init();

    /* init application */
    rt_application_init();

    /* user defined init function */
    rt_setup();

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    while (1);
}

/* RT_Thread instance */
RT_Thread RT_T;
