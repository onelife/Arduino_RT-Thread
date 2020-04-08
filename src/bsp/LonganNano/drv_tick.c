/***************************************************************************//**
 * @file    drv_tick.cpp
 * @brief   Arduino RT-Thread library RISC-V tick driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "include/rtthread.h"

#if defined(BOARD_SIPEED_LONGAN_NANO)

#include "bsp/bsp.h"

/***************************************************************************//**
 * @addtogroup LonganNano
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static rt_uint64_t overflow = 0;

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
rt_err_t bsp_hw_tick_init(void) {
    rt_uint64_t ticks;

    /* get counter value */
    ticks = *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIME);
    /* set compare value */
    if ((ticks + (TIMER_FREQ / RT_TICK_PER_SECOND)) < ticks) {
        /* overflow */
        overflow = ticks + (TIMER_FREQ / RT_TICK_PER_SECOND);
        *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = (rt_uint64_t)(-1);
    } else {
        overflow = 0;
        *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = \
            ticks + (TIMER_FREQ / RT_TICK_PER_SECOND);

    }
    *(rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MSIP) = 1;
    eclic_irq_enable(CLIC_INT_TMR, 0, 0);

    return RT_EOK;
}

/* system timer ISR */
void eclic_mtip_handler(void) {
    rt_uint64_t ticks;

    /* update compare value */
    if (overflow > 0) {
        /* overflow 2nd half */
        *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIME) = 0;
        *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = overflow;
        overflow = 0;
        return;
    }

    ticks = *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIME);
    if ((ticks + (TIMER_FREQ / RT_TICK_PER_SECOND)) < ticks) {
        /* overflow */
        overflow = ticks + (TIMER_FREQ / RT_TICK_PER_SECOND);
        *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = \
            (rt_uint64_t)(-1);
    } else {
        overflow = 0;
        *(rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = \
            ticks + (TIMER_FREQ / RT_TICK_PER_SECOND);
    }
    rt_interrupt_enter();
    rt_tick_increase();
    rt_interrupt_leave();
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(BOARD_SIPEED_LONGAN_NANO) */
