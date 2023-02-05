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
    rt_uint64_t ticks, compare;

    /* get counter value */
    ticks = get_timer_value();
    compare = ticks + (TIMER_FREQ / RT_TICK_PER_SECOND);
    /* set compare value */
    if (compare < ticks) {
        /* overflow */
        overflow = compare;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP + 4) = 0xffffffff;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = 0xffffffff;
        // *(volatile rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = (rt_uint64_t)(-1);
    } else {
        overflow = 0;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP + 4) = (rt_uint32_t)(compare >> 32);
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = (rt_uint32_t)(compare & 0xffffffff);
        // *(volatile rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = compare;

    }
    *(rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MSIP) = 1;
    /* set interrupt level and priority */
    eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
    eclic_irq_enable(CLIC_INT_TMR, 7, 1);

    return RT_EOK;
}

/* system timer ISR */
void eclic_mtip_handler(void) {
    rt_uint64_t ticks, compare;

    /* update compare value */
    if (overflow > 0) {
        /* overflow 2nd half */
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIME + 4) = 0;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIME) = 0;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP + 4) = (rt_uint32_t)(overflow >> 32);
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = (rt_uint32_t)(overflow & 0xffffffff);
        // *(volatile rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIME) = 0;
        // *(volatile rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = overflow;
        overflow = 0;
        return;
    }

    ticks = get_timer_value();
    compare = ticks + (TIMER_FREQ / RT_TICK_PER_SECOND);
    if (compare < ticks) {
        /* overflow */
        overflow = compare;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP + 4) = 0xffffffff;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = 0xffffffff;
        // *(volatile rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = (rt_uint64_t)(-1);
    } else {
        overflow = 0;
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP + 4) = (rt_uint32_t)(compare >> 32);
        *(volatile rt_uint32_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = (rt_uint32_t)(compare & 0xffffffff);
        // *(volatile rt_uint64_t *)(TIMER_CTRL_ADDR + TIMER_MTIMECMP) = compare;
    }
    rt_interrupt_enter();
    rt_tick_increase();
    rt_interrupt_leave();
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(BOARD_SIPEED_LONGAN_NANO) */
