/***************************************************************************//**
 * @file    drv_spi.h
 * @brief   Arduino RT-Thread library SPI device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

/* Includes ------------------------------------------------------------------*/
#include <SPI.h>    /* Arduino library */
#include "drv_common.h"

/* Exported defines ----------------------------------------------------------*/
#define SPI_DEFAULT_CONFIG          (SPI_CONFIG_MASTER)
#define SPI_DEFAULT_SPEED           (250000)
#define SPI_DEFAULT_RETRY           (3)
#define SPI_DEFAULT_LIMIT           (512)

#define SPI_CONFIG_MASTER           (1 << 0)    /* Master mode */

/* Exported types ------------------------------------------------------------*/
enum bsp_spi_channel {
    #if CONFIG_USING_SPI0
    CH0 = 0,
    #endif
    #if CONFIG_USING_SPI1
    CH1 = 1,
    #endif
    CH_NUM = CONFIG_USING_SPI0 + CONFIG_USING_SPI1,
};

struct bsp_spi_contex {
    rt_uint8_t chn;         /* channel number */
    rt_uint8_t cfg;         /* config */
    rt_uint32_t spd;        /* speed */
    SPISettings set;        /* setting */
    void *ldev;             /* lower level device (Arduino SPI) */
    struct rt_mutex lok;    /* lock */
    struct rt_device dev;   /* RT device */
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_spi_init(void);

#endif /* __DRV_SPI_H__ */
