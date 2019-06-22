/***************************************************************************//**
 * @file    drv_spiili.h
 * @brief   Arduino RT-Thread library ILI9341 device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPIILI_H__
#define __DRV_SPIILI_H__

/* Includes ------------------------------------------------------------------*/
#if CONFIG_USING_GUI
# include "include/rtgui.h"
#else
# define rtgui_color_t rt_uint32_t
#endif

/* Exported defines ----------------------------------------------------------*/
#define ILI_NAME                "ILI9341"
#define ILI_MAX_PARAM_LEN       (15)

/* commands */
#define ILI_CMD_NOP             0x00    // NOP
#define ILI_CMD_SWRESET         0x01    // Software Reset
#define ILI_CMD_RDDID           0x04    // Read Display Identification Information
#define ILI_CMD_RDDST           0x09    // Read Display Status

#define ILI_CMD_SLPIN           0x10    // Enter Sleep Mode
#define ILI_CMD_SLPOUT          0x11    // Sleep Out
#define ILI_CMD_PTLON           0x12    // Partial Mode ON
#define ILI_CMD_NORON           0x13    // Normal Display Mode ON

#define ILI_CMD_RDMODE          0x0A    // Read Display Power Mode
#define ILI_CMD_RDMADCTL        0x0B    // Read Display MADCTL
#define ILI_CMD_RDPIXFMT        0x0C    // Read Display Pixel Format
#define ILI_CMD_RDIMGFMT        0x0D    // Read Display Image Format
#define ILI_CMD_RDSELFDIAG      0x0F    // Read Display Self-Diagnostic Result

#define ILI_CMD_INVOFF          0x20    // Display Inversion OFF
#define ILI_CMD_INVON           0x21    // Display Inversion ON
#define ILI_CMD_GAMMASET        0x26    // Gamma Set
#define ILI_CMD_DISPOFF         0x28    // Display OFF
#define ILI_CMD_DISPON          0x29    // Display ON

#define ILI_CMD_CASET           0x2A    // Column Address Set
#define ILI_CMD_PASET           0x2B    // Page Address Set
#define ILI_CMD_RAMWR           0x2C    // Memory Write
#define ILI_CMD_RAMRD           0x2E    // Memory Read

#define ILI_CMD_PTLAR           0x30    // Partial Area
#define ILI_CMD_MADCTL          0x36    // Memory Access Control
#define ILI_CMD_VSCRSADD        0x37    // Vertical Scrolling Start Address
#define ILI_CMD_PIXFMT          0x3A    // COLMOD: Pixel Format Set

#define ILI_CMD_FRMCTR1         0xB1    // Frame Rate Control (In Normal Mode/Full Colors)
#define ILI_CMD_FRMCTR2         0xB2    // Frame Rate Control (In Idle Mode/8 colors)
#define ILI_CMD_FRMCTR3         0xB3    // Frame Rate control (In Partial Mode/Full Colors)
#define ILI_CMD_INVCTR          0xB4    // Display Inversion Control
#define ILI_CMD_DFUNCTR         0xB6    // Display Function Control

#define ILI_CMD_PWCTR1          0xC0    // Power Control 1
#define ILI_CMD_PWCTR2          0xC1    // Power Control 2
#define ILI_CMD_PWCTR3          0xC2    // Power Control 3
#define ILI_CMD_PWCTR4          0xC3    // Power Control 4
#define ILI_CMD_PWCTR5          0xC4    // Power Control 5
#define ILI_CMD_VMCTR1          0xC5    // VCOM Control 1
#define ILI_CMD_VMCTR2          0xC7    // VCOM Control 2

#define ILI_CMD_ID4             0xD3
#define ILI_CMD_RDID1           0xDA    // Read ID 1
#define ILI_CMD_RDID2           0xDB    // Read ID 2
#define ILI_CMD_RDID3           0xDC    // Read ID 3
#define ILI_CMD_RDID4           0xDD    // Read ID 4

#define ILI_CMD_GMCTRP1         0xE0    // Positive Gamma Correction
#define ILI_CMD_GMCTRN1         0xE1    // Negative Gamma Correction
// #define ILI_CMD_PWCTR6          0xFC
#define ILI_CMD_DELAY           0xFF    // (custom) delay

/* Exported types ------------------------------------------------------------*/
struct bsp_ili_contex {
    rt_uint16_t id;             /* ILI ID */
    rt_device_t ldev;           /* lower level device (RT SPI) */
    #if CONFIG_USING_GUI
    const struct rt_device_graphic_info *disp_info;
    const struct rtgui_graphic_driver_ops *disp_ops;
    #endif
    struct rt_device dev;       /* RT device */
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_spiIli_init(void);

#endif /* __DRV_SPIILI_H__ */
