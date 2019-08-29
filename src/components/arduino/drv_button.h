/***************************************************************************//**
 * @file    drv_button.h
 * @brief   Arduino RT-Thread library FT6202 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_BUTTON_H__
#define __DRV_BUTTON_H__

/* Includes ------------------------------------------------------------------*/
#include "drv_common.h"
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#endif

/* Exported defines ----------------------------------------------------------*/
#define BTN_NAME                        "BTN"
#define BTN_CHECK_TICK                  (4)     /* 40ms */

/* Exported types ------------------------------------------------------------*/
struct bsp_btn_contex {
    struct rt_device dev;               /* RT device */
    rt_uint32_t num;
    struct rt_timer tmr;
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_button_init(void);

#endif /* __DRV_BUTTON_H__ */
