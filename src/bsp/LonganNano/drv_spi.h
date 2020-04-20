/***************************************************************************//**
 * @file    drv_spi.h
 * @brief   Arduino RT-Thread library RISC-V SPI driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __LONGAN_NANO_DRV_SPI_H__
#define __LONGAN_NANO_DRV_SPI_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define SPI_MAX_SPEED              	(27000000)

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_spi_init(void);

#endif // __LONGAN_NANO_DRV_SPI_H__
