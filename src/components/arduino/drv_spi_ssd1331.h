/***************************************************************************//**
 * @file    drv_spi_ssd1331.h
 * @brief   Arduino RT-Thread library SSD1331 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPI_SSD1331_H__
#define __DRV_SPI_SSD1331_H__

/* Includes ------------------------------------------------------------------*/
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#else
# define rtgui_color_t rt_uint32_t
#endif

/* Exported defines ----------------------------------------------------------*/
#define SSD_NAME                    "SSD1331"
#define SSD1331_SPI_SPEED           (SPI_MAX_SPEED)

/* commands */
#define SSD1331_CMD_DRAWLINE        0x21
#define SSD1331_CMD_DRAWRECT        0x22
#define SSD1331_CMD_CLEAR           0x25
#define SSD1331_CMD_FILL            0x26
#define SSD1331_CMD_SETCOLUMN       0x15
#define SSD1331_CMD_SETROW          0x75
#define SSD1331_CMD_CONTRASTA       0x81
#define SSD1331_CMD_CONTRASTB       0x82
#define SSD1331_CMD_CONTRASTC       0x83
#define SSD1331_CMD_MASTERCURRENT   0x87
#define SSD1331_CMD_REMAP           0xA0
#define SSD1331_CMD_STARTLINE       0xA1
#define SSD1331_CMD_DISPLAYOFFSET   0xA2
#define SSD1331_CMD_NORMALDISPLAY   0xA4
#define SSD1331_CMD_DISPLAYALLON    0xA5
#define SSD1331_CMD_DISPLAYALLOFF   0xA6
#define SSD1331_CMD_INVERTDISPLAY   0xA7
#define SSD1331_CMD_MULTIPLEX       0xA8
#define SSD1331_CMD_MASTERCONFIG    0xAD
#define SSD1331_CMD_DISPLAYOFF      0xAE
#define SSD1331_CMD_DISPLAYON       0xAF
#define SSD1331_CMD_POWERMODE       0xB0
#define SSD1331_CMD_PRECHARGE       0xB1
#define SSD1331_CMD_CLOCKDIV        0xB3
#define SSD1331_CMD_PRECHARGEA      0x8A
#define SSD1331_CMD_PRECHARGEB      0x8B
#define SSD1331_CMD_PRECHARGEC      0x8C
#define SSD1331_CMD_PRECHARGELEVEL  0xBB
#define SSD1331_CMD_VCOMH           0xBE

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
rt_err_t bsp_hw_ssd1331_init(void);

#endif /* __DRV_SPI_SSD1331_H__ */
