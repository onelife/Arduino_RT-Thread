/***************************************************************************//**
 * @file    drv_spi_ssd1306.h
 * @brief   Arduino RT-Thread library ST7735 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPI_ST7735_CMD_H__
#define __DRV_SPI_ST7735_CMD_H__

/* Includes ------------------------------------------------------------------*/
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#else
# define rtgui_color_t rt_uint32_t
#endif

/* Exported defines ----------------------------------------------------------*/
#define ST7735_NAME                 "ST7735"
#define ST7735_SPI_SPEED            (4000000)

/* commands */
#define ST7735_CMD_RDDID            0x04
#define ST7735_CMD_SLPOUT           0x11
#define ST7735_CMD_INVON            0x21
#define ST7735_CMD_DISPON           0x29
#define ST7735_CMD_CASET            0x2A
#define ST7735_CMD_RASET            0x2B
#define ST7735_CMD_RAMWR            0x2C
#define ST7735_CMD_RAMRD            0x2E
#define ST7735_CMD_MADCTL           0x36
#define ST7735_CMD_COLMOD           0x3A
#define ST7735_CMD_FRMCTR1          0xB1
#define ST7735_CMD_FRMCTR2          0xB2
#define ST7735_CMD_FRMCTR3          0xB3
#define ST7735_CMD_INVCTR           0xB4
#define ST7735_CMD_PWCTR1           0xC0
#define ST7735_CMD_PWCTR2           0xC1
#define ST7735_CMD_PWCTR3           0xC2
#define ST7735_CMD_PWCTR4           0xC3
#define ST7735_CMD_PWCTR5           0xC4
#define ST7735_CMD_VMCTR1           0xC5
#define ST7735_CMD_GMCTRP1          0xE0
#define ST7735_CMD_GMCTRN1          0xE1
#define ST7735_CMD_DELAY            0xFF    // delay

/* Exported types ------------------------------------------------------------*/
struct bsp_disp_contex {
    rt_uint32_t id;
    struct rt_spi_device *ldev; /* lower level device (RT SPI Driver) */
    #if CONFIG_USING_GUI
    const struct rt_device_graphic_info *disp_info;
    const struct rtgui_graphic_driver_ops *disp_ops;
    #endif
    struct rt_device dev;       /* RT device */
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_st7735_init(void);

#endif /* __DRV_SPI_ST7735_CMD_H__ */
