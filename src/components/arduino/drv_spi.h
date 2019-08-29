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
#define SPI_DEFAULT_SPEED           (250000)
#ifdef ARDUINO_ARCH_SAM
# define SPI_MAX_SPEED              (24000000)
#else
# define SPI_MAX_SPEED              (12000000)
#endif
#define SPI_DEFAULT_RETRY           (10)
#define SPI_DEFAULT_LIMIT           (512)
#define SPI_FLAG_MORE               (rt_uint32_t)(0x01 << 16)
#define SPI_FLAG_READ_TOKEN(tk)     (rt_uint32_t)((tk & 0xff) << 8)
#define SPI_FLAG_IDLE_TOKEN(tk)     (rt_uint32_t)((tk & 0xff) << 0)

/* Exported types ------------------------------------------------------------*/
enum bsp_spi_channel {
    #if CONFIG_USING_SPI0
    SPI_CH0 = 0,
    #endif
    #if CONFIG_USING_SPI1
    SPI_CH1 = 1,
    #endif
    SPI_CH_NUM = CONFIG_USING_SPI0 + CONFIG_USING_SPI1,
};

struct bsp_spi_contex {
    rt_uint8_t chn;         /* channel number */
    rt_bool_t start;
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
