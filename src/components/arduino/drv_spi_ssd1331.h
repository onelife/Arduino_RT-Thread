/***************************************************************************//**
 * @file    drv_spi_ssd1331.h
 * @brief   Arduino RT-Thread library SSD1331 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPI_SSD_H__
#define __DRV_SPI_SSD_H__

/* Includes ------------------------------------------------------------------*/
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#else
# define rtgui_color_t rt_uint32_t
#endif

/* Exported defines ----------------------------------------------------------*/
#define SSD_NAME                "SSD1331"
#define SSD_MAX_PARAM_LEN       (7)

/* commands */
#define SSD_CMD_DRAWLINE        0x21
#define SSD_CMD_DRAWRECT        0x22
#define SSD_CMD_CLEAR           0x25
#define SSD_CMD_FILL            0x26
#define SSD_CMD_SETCOLUMN       0x15
#define SSD_CMD_SETROW          0x75
#define SSD_CMD_CONTRASTA       0x81
#define SSD_CMD_CONTRASTB       0x82
#define SSD_CMD_CONTRASTC       0x83
#define SSD_CMD_MASTERCURRENT   0x87
#define SSD_CMD_SETREMAP        0xA0
#define SSD_CMD_STARTLINE       0xA1
#define SSD_CMD_DISPLAYOFFSET   0xA2
#define SSD_CMD_NORMALDISPLAY   0xA4
#define SSD_CMD_DISPLAYALLON    0xA5
#define SSD_CMD_DISPLAYALLOFF   0xA6
#define SSD_CMD_INVERTDISPLAY   0xA7
#define SSD_CMD_SETMULTIPLEX    0xA8
#define SSD_CMD_SETMASTER       0xAD
#define SSD_CMD_DISPLAYOFF      0xAE
#define SSD_CMD_DISPLAYON       0xAF
#define SSD_CMD_POWERMODE       0xB0
#define SSD_CMD_PRECHARGE       0xB1
#define SSD_CMD_CLOCKDIV        0xB3
#define SSD_CMD_PRECHARGEA      0x8A
#define SSD_CMD_PRECHARGEB      0x8B
#define SSD_CMD_PRECHARGEC      0x8C
#define SSD_CMD_PRECHARGELEVEL  0xBB
#define SSD_CMD_VCOMH           0xBE
#define SSD_CMD_DELAY           0xFF    // (custom) delay

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

#endif /* __DRV_SPI_SSD_H__ */
