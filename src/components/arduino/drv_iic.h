/***************************************************************************//**
 * @file    drv_iic.h
 * @brief   Arduino RT-Thread library IIC device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_IIC_H__
#define __DRV_IIC_H__

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>   /* Arduino library */

/* Exported defines ----------------------------------------------------------*/
#define IIC_FLAG_SLAVE_ADDR(ad)     (rt_uint32_t)((ad & 0xff) << 0)
#define IIC_FLAG_ACCESS_ADDR(ad)    (rt_uint32_t)((ad & 0xff) << 8)

/* Exported types ------------------------------------------------------------*/
enum bsp_iic_channel {
    #if CONFIG_USING_IIC0
    IIC_CH0 = 0,
    #endif
    #if CONFIG_USING_IIC1
    IIC_CH1 = 1,
    #endif
    IIC_CH_NUM = CONFIG_USING_IIC0 + CONFIG_USING_IIC1,
};

struct bsp_iic_contex {
    rt_uint8_t chn;         /* channel number */
    // rt_bool_t start;
    // rt_uint8_t addr;
    void *ldev;             /* lower level device (Arduino IIC) */
    struct rt_mutex lok;    /* lock */
    struct rt_device dev;   /* RT device */
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_iic_init(void);

#endif /* __DRV_IIC_H__ */
