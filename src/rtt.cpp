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


#define KERNEL_PRIORITY ((CONFIG_KERNEL_PRIORITY << (8 - __NVIC_PRIO_BITS)) & 0xff)
#define TICK_COUNT      (CONFIG_TICK_PER_SECOND / RT_TICK_PER_SECOND)


extern "C" {
    #ifdef RT_DEBUG
        void assert_failed(uint8_t * file, uint32_t line) {
            pinMode(LED_BUILTIN, OUTPUT);
            digitalWrite(LED_BUILTIN, HIGH);
            rt_kprintf("\r\nWrong parameter value detected on\r\n");
            rt_kprintf("\tfile  %s\r\n", file);
            rt_kprintf("\tline  %d\r\n", line);

            while (1) {
            }
        }
    #endif

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
        #if (!CONFIG_NO_CONSOLE)
            if (NULL != str) Serial.print(str);
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

        if (serialEventRun) serialEventRun();
        return 0;
    }
}


void rt_hw_board_init(void) __attribute__((weak));
void rt_hw_board_init(void) { }

void rt_application_init(void) __attribute__((weak));
void rt_application_init(void) { }


void RT_Thread::begin(void) {
    #if defined( ARDUINO_SAM_DUE)
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

    /* init board */
    rt_hw_board_init();

    #ifdef RT_USING_HEAP
        /* init memory management */
        rt_system_heap_init((void *)&rtt_heap, (void *)&rtt_heap[CONFIG_HEAP_SIZE-1]);
    #endif

    /* init tick */
    rt_system_tick_init();

    /* init kernel object */
    rt_system_object_init();

    /* init timer */
    rt_system_timer_init();

    /* init scheduler */
    rt_system_scheduler_init();

    /* show version */
    rt_show_version();

    /* init timer thread */
    rt_system_timer_thread_init();

    /* init idle thread */
    rt_thread_idle_init();

    /* init application */
    rt_application_init();

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    return;
}

/* RT_Thread instance */
RT_Thread RT_T;
