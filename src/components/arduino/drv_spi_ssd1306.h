/***************************************************************************//**
 * @file    drv_spi_ssd1306.h
 * @brief   Arduino RT-Thread library SSD1306 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPI_SSD1306_CMD_H__
#define __DRV_SPI_SSD1306_CMD_H__

/* Includes ------------------------------------------------------------------*/
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#else
# define rtgui_color_t rt_uint32_t
#endif

/* Exported defines ----------------------------------------------------------*/
#define SSD_NAME                    "SSD1306"
#define SSD1306_SPI_SPEED           (4000000)

/* commands */
#define SSD1306_CMD_ADDRMODE        0x20
#define SSD1306_CMD_SETCOLUMN       0x21
#define SSD1306_CMD_SETPAGE         0x22
#define SSD1306_CMD_STARTLINE       0x40    // 0 ~ 63
#define SSD1306_CMD_SCROLLOFF       0x2E    // Stop scroll
#define SSD1306_CMD_SCROLLON        0x2F    // Start scroll
#define SSD1306_CMD_CONTRAST        0x81    // 0~255
#define SSD1306_CMD_CHARGEPUMP      0x8D
#define SSD1306_CMD_SEGREMAP        0xA0
#define SSD1306_CMD_DISPLAYALLON    0xA4    // A4/A5
#define SSD1306_CMD_NORMALDISPLAY   0xA6
#define SSD1306_CMD_INVERTDISPLAY   0xA7
#define SSD1306_CMD_MULTIPLEX       0xA8
#define SSD1306_CMD_DISPLAYOFF      0xAE
#define SSD1306_CMD_DISPLAYON       0xAF
#define SSD1306_CMD_COMSCANDIR      0xC8    // COM63 to COM0
#define SSD1306_CMD_DISPLAYOFFSET   0xD3
#define SSD1306_CMD_CLOCKDIV        0xD5
#define SSD1306_CMD_PRECHARGE       0xD9
#define SSD1306_CMD_COMPINS         0xDA
#define SSD1306_CMD_VCOMDETECT      0xDB

/* Exported types ------------------------------------------------------------*/
struct bsp_ssd_contex {
    rt_device_t ldev;           /* lower level device (RT SPI) */
    #if CONFIG_USING_GUI
    const struct rt_device_graphic_info *disp_info;
    const struct rtgui_graphic_driver_ops *disp_ops;
    #endif
    struct rt_device dev;       /* RT device */
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_ssd1306_init(void);

#endif /* __DRV_SPI_SSD1306_CMD_H__ */
