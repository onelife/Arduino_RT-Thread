/***************************************************************************//**
 * @file    drv_spi_ssd1306.cpp
 * @brief   Arduino RT-Thread library SSD1306 device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_SSD1306
}

#include <Arduino.h>
#include <SPI.h>    /* Arduino library */

#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

extern "C" {

#include "drv_spi.h"
#include "drv_spi_ssd1306.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_SSD_DEBUG
#  define LOG_LVL                   LOG_LVL_DBG
# else
#  define LOG_LVL                   LOG_LVL_INFO
# endif
# define LOG_TAG                    "SSD"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# ifdef BSP_SPI_DEBUG
#  define LOG_D(format, args...)    rt_kprintf(format "\n", ##args)
# else
#  define LOG_D(format, args...)
# endif
# define LOG_I                      LOG_D
# define LOG_HEX(format, args...)                    
#endif /* RT_USING_ULOG */

#define SSD_IO_INIT()               {   \
    pinMode(CONFIG_SSD_CS_PIN, OUTPUT); \
    pinMode(CONFIG_SSD_DC_PIN, OUTPUT); \
    pinMode(CONFIG_SSD_RST_PIN, OUTPUT); \
    digitalWrite(CONFIG_SSD_CS_PIN, HIGH); \
    digitalWrite(CONFIG_SSD_DC_PIN, HIGH); \
    digitalWrite(CONFIG_SSD_RST_PIN, HIGH); \
}
#define SSD_RESET()                 digitalWrite(CONFIG_SSD_RST_PIN, LOW); \
                                    delay(10); \
                                    digitalWrite(CONFIG_SSD_RST_PIN, HIGH); \
                                    delay(1)
#define SSD_START()                 digitalWrite(CONFIG_SSD_CS_PIN, LOW); \
                                    LOG_D("[1306] cs clear")
#define SSD_STOP()                  digitalWrite(CONFIG_SSD_CS_PIN, HIGH); \
                                    LOG_D("[1306] cs set")
#define SSD_CMD_START()             digitalWrite(CONFIG_SSD_DC_PIN, LOW); \
                                    LOG_D("[1306] dc clear")
#define SSD_CMD_STOP()              digitalWrite(CONFIG_SSD_DC_PIN, HIGH); \
                                    LOG_D("[1306] dc set")
#define _NUM_TO_STR(n)              #n
#define _CH_TO_STR(ch)              _NUM_TO_STR(ch)
#define SSD_LOWER_DEVICE_NAME       "SPI" _CH_TO_STR(CONFIG_SSD_SPI_CHANNEL)
#define SSD_CTX()                   (&ssd_ctx)
#if CONFIG_USING_GUI
# define SSD_BUF()                  (ssd_ctx.disp_info->framebuffer)
# define SSD_WIDTH()                (ssd_ctx.disp_info->width)
# define SCOPE                      static
#else
# define SSD_WIDTH()                (CONFIG_GUI_WIDTH)
# define SSD_BUF()                  (_framebuffer)
# define SCOPE
#endif

/* Private function prototypes -----------------------------------------------*/
#if CONFIG_USING_GUI
static void ssd_set_pixel(rtgui_color_t *c, int x, int y);
static void ssd_get_pixel(rtgui_color_t *c, int x, int y);
static void ssd_draw_hline(rtgui_color_t *c, int x1, int x2, int y);
static void ssd_draw_vline(rtgui_color_t *c, int x , int y1, int y2);
static void ssd_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y);
#endif

/* Private constants ---------------------------------------------------------*/
static const rt_uint8_t init_code[] = {
    SSD1306_CMD_DISPLAYOFF,
    SSD1306_CMD_CLOCKDIV,           0x80,   // Freq, CLK Div Ratio
    SSD1306_CMD_CHARGEPUMP,         0x14,   // Enable
    SSD1306_CMD_PRECHARGE,          0xf1,   // Phase1: 1, Phase2: 15
    SSD1306_CMD_VCOMDETECT,         0x30,   // 0.83x Vcc
    SSD1306_CMD_CONTRAST,           0xcf,
    SSD1306_CMD_ADDRMODE,           0x00,   // Page addr mode
    SSD1306_CMD_SEGREMAP | 0x01,            // Col127 => Seg0
    SSD1306_CMD_MULTIPLEX,          CONFIG_GUI_HIGH - 1,
    SSD1306_CMD_COMPINS,            0x12,   // Alternative, Disable L2R
    SSD1306_CMD_COMSCANDIR,
    SSD1306_CMD_DISPLAYOFFSET,      0x00,
    SSD1306_CMD_STARTLINE | 0x00,
    SSD1306_CMD_DISPLAYALLON,
    SSD1306_CMD_NORMALDISPLAY,
    SSD1306_CMD_SCROLLOFF,
    SSD1306_CMD_DISPLAYON,
};

static rt_uint8_t _framebuffer[CONFIG_GUI_WIDTH * (CONFIG_GUI_HIGH / 8)];

#if CONFIG_USING_GUI
static const struct rt_device_graphic_info disp_info = {
    .pixel_format   = RTGRAPHIC_PIXEL_FORMAT_MONO,
    .bits_per_pixel = 1,
    .reserved       = 0,
    .width          = CONFIG_GUI_WIDTH,
    .height         = CONFIG_GUI_HIGH,
    .framebuffer    = _framebuffer,
};

static struct rtgui_graphic_driver_ops disp_ops = {
    .set_pixel      = ssd_set_pixel,
    .get_pixel      = ssd_get_pixel,
    .draw_hline     = ssd_draw_hline,
    .draw_vline     = ssd_draw_vline,
    .draw_raw_hline = ssd_draw_raw_hline,
};
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_ssd_contex ssd_ctx;

/* Private functions ---------------------------------------------------------*/
static rt_err_t ssd_write_data(rt_device_t ldev, rt_uint8_t *data,
    rt_uint32_t len) {
    rt_uint8_t buf_ins[RT_ALIGN(5, RT_ALIGN_SIZE)];
    rt_err_t ret;

    ret = RT_EOK;
    LOG_D("[1306] write data [%d]", len);

    do {
         rt_size_t ret_len;

        /* send data
            - color in mono format
         */
        /* build inst
            - inst len: 0
            - inst: none
            - tx buf addr: offset align with RT_ALIGN_SIZE
         */
        buf_ins[0] = 0;
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        *(rt_uint8_t **)(&buf_ins[RT_ALIGN(1, RT_ALIGN_SIZE)]) = data;
        #pragma GCC diagnostic pop

        ret_len = rt_device_write(ldev, 0, buf_ins, len);
        if (len != ret_len) {
            LOG_E("[1306 E] write data failed! [%d]", ret_len);
            ret = -RT_EIO;
            break;
        }
    } while (0);

    return ret;
}

static void ssd_set_window(rt_uint8_t x1, rt_uint8_t x2, rt_uint8_t y1,
    rt_uint8_t y2) {
    rt_uint8_t data[6];

    /* set x */
    data[0] = SSD1306_CMD_SETCOLUMN;
    if (x1 > x2) {
        data[1] = x2;
        data[2] = x1;
    } else {
        data[1] = x1;
        data[2] = x2;
    }
    /* set y */
    data[3] = SSD1306_CMD_SETPAGE;
    if (y1 > y2) {
        data[4] = y2 / 8;
        data[5] = y1 / 8;
    } else {
        data[4] = y1 / 8;
        data[5] = y2 / 8;
    }
    SSD_CMD_START();
    (void)ssd_write_data(SSD_CTX()->ldev, data, sizeof(data));
    SSD_CMD_STOP();
    LOG_D("[1306] win (%d,%d) (%d,%d)", x1, y1, x2, y2);
}

SCOPE void ssd_set_pixel(rtgui_color_t *c, int x, int y) {
    rt_uint8_t x1 = x & 0x000000ff;
    rt_uint8_t y1 = y & 0x000000ff;
    rt_uint8_t *ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];

    if (*c)
        *ptr |=  ((rt_uint8_t)1 << (y1 & 0x07));
    else
        *ptr &= ~((rt_uint8_t)1 << (y1 & 0x07));

    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_set_window(x1, x1, y1, y1);
    (void)ssd_write_data(SSD_CTX()->ldev, ptr, 1);
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[1306] set pixel %04x (%d, %d)", *c, x, y);
}

SCOPE void ssd_get_pixel(rtgui_color_t *c, int x, int y) {
    rt_uint8_t x1 = x & 0x000000ff;
    rt_uint8_t y1 = y & 0x000000ff;
    rt_uint8_t *ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];

    if (*ptr & (1 << (y1 & 0x07)))
        *c = 0xffffffff;
    else
        *c = 0xff000000;
    LOG_D("[1306] get pixel %08x (%d, %d)", *c, x, y);
}

SCOPE void ssd_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y) {
    rt_uint8_t y1 = y & 0x000000ff;
    rt_uint8_t *ptr;
    rt_uint8_t mask, i, len;

    x1 &= 0x000000ff;
    x2 &= 0x000000ff;
    ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];
    mask = 1 << (y1 & 0x07);
    len = x2 - x1 + 1;
    for (i = 0; i < len; i++, ptr++) {
        if (*(pixels + (i / 8)) & (1 << (i % 8)))
            *ptr |=  mask;
        else
            *ptr &= ~mask;
    }

    ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];
    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_set_window(x1, x2, y1, y1);
    (void)ssd_write_data(SSD_CTX()->ldev, ptr, len);
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[1306] raw hline (%d - %d, %d)", x1, x2, y);
}

SCOPE void ssd_draw_hline(rtgui_color_t *c, int x1, int x2, int y) {
    rt_uint8_t y1 = y & 0x000000ff;
    rt_uint8_t *ptr;
    rt_uint8_t mask, i, len;

    x1 &= 0x000000ff;
    x2 &= 0x000000ff;
    ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];
    mask = 1 << (y1 & 0x07);
    len = x2 - x1 + 1;
    if (*c) {
        for (i = 0; i < len; i++, ptr++)
            *ptr |=  mask;
    } else {
        for (i = 0; i < len; i++, ptr++)
            *ptr &= ~mask;
    }

    ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];
    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_set_window(x1, x2, y1, y1);
    (void)ssd_write_data(SSD_CTX()->ldev, ptr, len);
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[1306] hline %04x (%d - %d, %d)", *c, x1, x2, y);
}

SCOPE void ssd_draw_vline(rtgui_color_t *c, int x , int y1, int y2) {
    rt_uint8_t x1 = x & 0x000000ff;
    rt_uint8_t *ptr;
    rt_uint8_t mask, y, len;

    y1 &= 0x000000ff;
    y2 &= 0x000000ff;
    ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];
    len = 1;
    if (*c) {
        for (y = y1; y < y2; y++) {
            mask = 1 << (y % 8);
            *ptr |=  mask;
            if (mask == 0x80) {
                ptr++;
                len++;
            }
        }
    } else {
        for (y = y1; y < y2; y++) {
            mask = 1 << (y % 8);
            *ptr &= ~mask;
            if (mask == 0x80) {
                ptr++;
                len++;
            }
        }
    }

    ptr = &SSD_BUF()[(y1 >> 3) * SSD_WIDTH() + x1];
    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_set_window(x1, x1, y1, y2);
    (void)ssd_write_data(SSD_CTX()->ldev, ptr, len);
    SSD_STOP();
    LOG_D("[1306] vline %04x (%d, %d - %d)", *c, x, y1, y2);
}

static rt_err_t bsp_ssd1306_init(rt_device_t dev) {
    struct bsp_ssd_contex *ctx = SSD_CTX();
    rt_err_t ret;
    (void)dev;

    rt_memset(_framebuffer, 0x00, sizeof(_framebuffer));
    /* open lower level device */
    ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != ret) return ret;
    SSD_START();

    do {
        /* init code*/
        SSD_CMD_START();
        ret = ssd_write_data(ctx->ldev, (rt_uint8_t *)init_code,
            sizeof(init_code));
        SSD_CMD_STOP();
        if (RT_EOK != ret) break;

        LOG_D("[1306] init ok");
    } while (0);

    SSD_STOP();
    rt_device_close(ctx->ldev);

    if (RT_EOK != ret) {
        LOG_W("[1306 E] init failed! (%08x)", ret);
    }
    return ret;
}

static rt_err_t bsp_ssd1306_control(rt_device_t dev, rt_int32_t cmd,
    void *args) {
    struct bsp_ssd_contex *ctx = SSD_CTX();
    rt_err_t ret = -RT_ERROR;
    #if !CONFIG_USING_GUI
        (void)args;
    #endif
    (void)dev;

    switch (cmd) {
    case RTGRAPHIC_CTRL_RECT_UPDATE:
    {
        // struct rt_device_rect_info *rect = (struct rt_device_rect_info *)args;
        // LOG_D("update: %d %d, %d %d", rect->x, rect->y, rect->width,
        //     rect->height);

        ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
        if (RT_EOK != ret) return ret;
        SSD_START();
        ssd_set_window(0, CONFIG_GUI_WIDTH - 1, 0, CONFIG_GUI_HIGH - 1);
        (void)ssd_write_data(SSD_CTX()->ldev, _framebuffer,
            sizeof(_framebuffer));
        SSD_STOP();
        rt_device_close(SSD_CTX()->ldev);
        break;
    }

    case RTGRAPHIC_CTRL_POWERON:
    case RTGRAPHIC_CTRL_POWEROFF:
        break;

    case RTGRAPHIC_CTRL_GET_INFO:
        #if CONFIG_USING_GUI
        rt_memcpy(args, ctx->disp_info, sizeof(struct rt_device_graphic_info));
        ret = RT_EOK;
        #endif
        break;

    case RTGRAPHIC_CTRL_SET_MODE:
    case RTGRAPHIC_CTRL_GET_EXT:
        break;

    case RT_DEVICE_CTRL_CURSOR_SET_POSITION:
    case RT_DEVICE_CTRL_CURSOR_SET_TYPE:
        #ifdef RTGUI_USING_HW_CURSOR
        #endif
        break;

    default:
        break;
    }

    return ret;
}

/***************************************************************************//**
 * @brief Initialize SSD contex
 *
 * @param[in] struct bsp_ssd_contex *ctx - Pointer to SSD contex
 *
 * @param[in] const char *name - Pointer to SSD name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_ssd1306_contex_init(struct bsp_ssd_contex *ctx,
    const char *name, rt_device_t ldev) {
    rt_err_t ret;

    do {
        ctx->ldev = ldev;
        #if CONFIG_USING_GUI
            /* set gui device */
            ctx->disp_info = &disp_info;
            ctx->disp_ops = &disp_ops;
        #endif

        /* register device */
        ctx->dev.type = RT_Device_Class_Graphic;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = bsp_ssd1306_init;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = RT_NULL;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_ssd1306_control;
        #if CONFIG_USING_GUI
            /* RTT-GUI will get ops here */
            ctx->dev.user_data = (void *)&disp_ops;
        #endif
        ret = rt_device_register(&ctx->dev, name,
            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
    } while (0);

    return ret;
}

/* Public functions ----------------------------------------------------------*/
/***************************************************************************//**
 * @brief - initialize SSD contex and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_ssd1306_init(void) {
    rt_err_t ret;

    do {
        rt_device_t ldev;

        /* get lower level device */
        ldev = rt_device_find(SSD_LOWER_DEVICE_NAME);
        if (RT_NULL == ldev) {
            LOG_D("[1306] can't find device %s!", SSD_LOWER_DEVICE_NAME);
            ret = -RT_ERROR;
            break;
        }
        LOG_D("[1306] find device %s", SSD_LOWER_DEVICE_NAME);

        /* switch to ssd1306 speed */
        ret = rt_device_control(ldev, RT_DEVICE_CTRL_SPI_SPEED,
            (void *)SSD1306_SPI_SPEED);
        if (RT_EOK != ret) break;

        ret = bsp_ssd1306_contex_init(SSD_CTX(), SSD_NAME, ldev);
        if (RT_EOK != ret) break;

        SSD_IO_INIT();
        SSD_RESET();
        LOG_D("[1306] h/w init ok");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[1306] h/w init failed: %d", ret);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_SSD1306 */

} /* extern "C" */
