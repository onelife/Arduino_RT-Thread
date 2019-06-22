/***************************************************************************//**
 * @file    drv_iic_ft6202.h
 * @brief   Arduino RT-Thread library FT6202 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_IIC_FT6206_H__
#define __DRV_IIC_FT6206_H__

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>   /* Arduino library */

#include "drv_common.h"
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#endif

/* Exported defines ----------------------------------------------------------*/
#define FT6206_NAME                     "FT6206"
#define FT6206_ADDR                     (0x38)
#define FT6206_TOUCH_NUM                (2)
#define DEFAULT_THRESHOLD               (40)
#define DEFAULT_INTERRUPT_MODE          (0x01)

#define FT6206_CHIPID                   (0x06)
#define FT6206_VENDID                   (0x11)

#define FT6206_REG_THRESHHOLD           (0x80)
#define FT6206_REG_CHIPID               (0xA3)      /* Chip Selecting */
#define FT6206_REG_G_MODE               (0xA4)      /* Interrupt mode */
#define FT6206_REG_FIRMID               (0xA6)      /* Firmware Version */
#define FT6206_REG_VENDID               (0xA8)      /* FocalTech’s Panel ID */

/* Exported types ------------------------------------------------------------*/
struct bsp_ft_contex {
    rt_uint16_t id;                     /* FT6206 ID */
    rt_device_t ldev;                   /* lower level device (RT IIC) */
    struct rt_device dev;               /* RT device */
    rt_uint8_t touch_num;
    rt_uint8_t gesture;
    #if CONFIG_USING_GUI
        rtgui_touch_t touchs[FT6206_TOUCH_NUM];
    #endif
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_ft6206_init(void);

#endif /* __DRV_IIC_FT6206_H__ */
