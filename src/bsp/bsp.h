/***************************************************************************//**
 * @file    bsp.h
 * @brief   Arduino RT-Thread library BSP header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __BSP_H__
#define __BSP_H__

#include "include/rtthread.h"

#ifdef BOARD_SIPEED_LONGAN_NANO
# include "system_gd32vf103.h"
# include "riscv_encoding.h"
# include "n200_timer.h"
# include "bsp/LonganNano/drv_tick.h"
# if CONFIG_USING_DRIVER_SERIAL
#  include "gd32vf103_usart.h"
#  include "bsp/LonganNano/drv_usart.h"
# endif /* CONFIG_USING_DRIVER_SERIAL */
# if CONFIG_USING_DRIVER_SPI
#  include "gd32vf103_spi.h"
#  include "bsp/LonganNano/drv_spi.h"
# endif /* CONFIG_USING_DRIVER_SERIAL */
#endif /* BOARD_SIPEED_LONGAN_NANO */

#endif /* __BSP_H__ */
