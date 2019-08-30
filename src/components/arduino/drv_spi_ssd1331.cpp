/***************************************************************************//**
 * @file    drv_spi_ssd1331.cpp
 * @brief   Arduino RT-Thread library SSD1331 device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_SSD1331
}

#include <Arduino.h>
#include <SPI.h>    /* Arduino library */

#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

extern "C" {

#include "drv_spi.h"
#include "drv_spi_ssd1331.h"

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
    pinMode(CONFIG_SSD_PWR_PIN, OUTPUT); \
    digitalWrite(CONFIG_SSD_CS_PIN, HIGH); \
    digitalWrite(CONFIG_SSD_DC_PIN, HIGH); \
    digitalWrite(CONFIG_SSD_RST_PIN, HIGH); \
    digitalWrite(CONFIG_SSD_PWR_PIN, LOW); \
}
#define SSD_RESET()                 digitalWrite(CONFIG_SSD_RST_PIN, LOW); \
                                    delay(5); \
                                    digitalWrite(CONFIG_SSD_RST_PIN, HIGH); \
                                    delay(10)
#define SSD_LIGHT_ON()              digitalWrite(CONFIG_SSD_PWR_PIN, HIGH)
#define SSD_LIGHT_OFF()             digitalWrite(CONFIG_SSD_PWR_PIN, LOW)
#define SSD_START()                 digitalWrite(CONFIG_SSD_CS_PIN, LOW); \
                                    LOG_D("[SSD] start")
#define SSD_STOP()                  digitalWrite(CONFIG_SSD_CS_PIN, HIGH); \
                                    LOG_D("[SSD] stop")
#define SSD_CMD_START()             digitalWrite(CONFIG_SSD_DC_PIN, LOW)
#define SSD_CMD_STOP()              digitalWrite(CONFIG_SSD_DC_PIN, HIGH)
#define _NUM_TO_STR(n)              #n
#define _CH_TO_STR(ch)              _NUM_TO_STR(ch)
#define SSD_LOWER_DEVICE_NAME       "SPI" _CH_TO_STR(CONFIG_SSD_SPI_CHANNEL)
#define SSD_CTX()                   (&ssd_ctx)
#if CONFIG_USING_GUI
# define SCOPE                      static
#else
# define SCOPE
#endif

/* Private function prototypes -----------------------------------------------*/
#if CONFIG_USING_GUI
static void ssd_set_pixel(rtgui_color_t *c, int x, int y);
static void ssd_draw_hline(rtgui_color_t *c, int x1, int x2, int y);
static void ssd_draw_vline(rtgui_color_t *c, int x , int y1, int y2);
static void ssd_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y);
#endif

/* Private constants ---------------------------------------------------------*/
static const rt_uint8_t init_code[] = {
    SSD_CMD_DISPLAYOFF,     0,
    SSD_CMD_SETREMAP,       1,  0x60,
    SSD_CMD_STARTLINE,      1,  0x00,
    SSD_CMD_DISPLAYOFFSET,  1,  0x00,
    SSD_CMD_NORMALDISPLAY,  0,
    SSD_CMD_SETMULTIPLEX,   1,  0x3F,   // 1/64 duty
    SSD_CMD_SETMASTER,      1,  0x8E,
    SSD_CMD_POWERMODE,      1,  0x0B,
    SSD_CMD_PRECHARGE,      1,  0x31,
    SSD_CMD_CLOCKDIV,       1,  0xF0,   // Oscillator Frequency, CLK Div Ratio
    SSD_CMD_PRECHARGEA,     1,  0x64,
    SSD_CMD_PRECHARGEB,     1,  0x78,
    SSD_CMD_PRECHARGEC,     1,  0x64,
    SSD_CMD_PRECHARGELEVEL, 1,  0x3A,
    SSD_CMD_VCOMH,          1,  0x3E,
    SSD_CMD_CONTRASTA,      1,  0x91,
    SSD_CMD_CONTRASTB,      1,  0x50,
    SSD_CMD_CONTRASTC,      1,  0x7D,
    SSD_CMD_MASTERCURRENT,  1,  0x08,
    SSD_CMD_CLEAR,          4,  0x00, 0x00, 95, 63,
    SSD_CMD_DISPLAYON,      0,
    0x00                                // End
};

#if CONFIG_USING_GUI
static const struct rt_device_graphic_info disp_info = {
    .pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565,
    .bits_per_pixel = 16,
    .reserved       = 0,
    .width          = CONFIG_GUI_WIDTH,
    .height         = CONFIG_GUI_HIGH,
    .framebuffer    = RT_NULL,
};

static struct rtgui_graphic_driver_ops disp_ops = {
    .set_pixel      = ssd_set_pixel,
    .get_pixel      = RT_NULL,
    .draw_hline     = ssd_draw_hline,
    .draw_vline     = ssd_draw_vline,
    .draw_raw_hline = ssd_draw_raw_hline,
};
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_ssd_contex ssd_ctx;

/* Private functions ---------------------------------------------------------*/
static rt_err_t ssd_write_cmd(rt_device_t ldev, rt_uint8_t cmd,
    rt_uint8_t *param, rt_uint8_t len) {
    rt_uint8_t buf_ins[RT_ALIGN(6, RT_ALIGN_SIZE)];
    rt_err_t ret;

    if (len > SSD_MAX_PARAM_LEN) {
        LOG_E("[SSD E] param too long");
        return -RT_ERROR;
    } else if ((len > 0) && !param) {
        LOG_E("[SSD E] bad param");
        return -RT_EINVAL;
    }
    ret = RT_EOK;
    LOG_D("[SSD] send cmd %02x [%d]", cmd, len);

    SSD_CMD_START();
    do {
        /* send cmd and param */
        /* build inst
            - inst len: 1
            - inst: cmd
            - param buf addr: offset align with RT_ALIGN_SIZE
         */
        buf_ins[0] = 1;
        buf_ins[1] = cmd;
        if (len) {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(rt_uint8_t **)(&buf_ins[ \
                RT_ALIGN(buf_ins[0] + 1, RT_ALIGN_SIZE)]) = param;
            #pragma GCC diagnostic pop
        }
        if (len != rt_device_write(ldev, 0, buf_ins, len)) {
            LOG_W("[SSD E] send cmd failed!");
            ret = -RT_EIO;
            break;
        }
    } while (0);
    SSD_CMD_STOP();

    return ret;
}

static rt_err_t ssd_write_data(rt_device_t ldev, rt_uint8_t *data,
    rt_uint32_t len) {
    rt_uint8_t buf_ins[RT_ALIGN(5, RT_ALIGN_SIZE)];
    rt_err_t ret;

    ret = RT_EOK;
    LOG_D("[SSD] write data [%d]", len);

    do {
         rt_size_t ret_len;

        /* send param and receive resp
            - color in RGB565 (RRRRRGGG GGGBBBBB) format
            - send MSB first (RRRRRGGG)
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
            LOG_E("[SSD E] write data failed! [%d]", ret_len);
            ret = -RT_EIO;
            break;
        }
    } while (0);

    return ret;
}

static void ssd_set_window(rt_uint8_t x1, rt_uint8_t x2, rt_uint8_t y1,
    rt_uint8_t y2) {
    rt_uint8_t data[2];

    /* set x */
    if (x1 > x2) {
        data[0] = x2;
        data[1] = x1;
    } else {
        data[0] = x1;
        data[1] = x2;
    }
    (void)ssd_write_cmd(SSD_CTX()->ldev, SSD_CMD_SETCOLUMN, data, 2);

    /* set y */
    if (y1 > y2) {
        data[0] = y2;
        data[1] = y1;
    } else {
        data[0] = y1;
        data[1] = y2;
    }
    (void)ssd_write_cmd(SSD_CTX()->ldev, SSD_CMD_SETROW, data, 2);
    LOG_D("[SSD] win (%d,%d) (%d,%d)", x1, y1, x2, y2);
}

static void ssd_draw_line(rtgui_color_t *c, rt_uint8_t x1, rt_uint8_t x2,
    rt_uint8_t y1, rt_uint8_t y2) {
    rt_uint8_t data[7];

    if (x1 > x2) {
        data[0] = x2;
        data[2] = x1;
    } else {
        data[0] = x1;
        data[2] = x2;
    }
    if (y1 > y2) {
        data[1] = y2;
        data[3] = y1;
    } else {
        data[1] = y1;
        data[3] = y2;
    }
    /* color in:  GGGBBBBB RRRRRGGG
       color out: 00RRRRR0 00GGGGGG 00BBBB0 */
    data[4] = (rt_uint8_t)((*c & 0x00f8) >> 2);
    data[5] = (rt_uint8_t)(((*c & 0x0007) << 3) | ((*c & 0xe000) >> 13));
    data[6] = (rt_uint8_t)((*c & 0x1f00) >> 7);
    (void)ssd_write_cmd(SSD_CTX()->ldev, SSD_CMD_DRAWLINE, data, 7);
    LOG_D("[SSD] line %04x (%d,%d) (%d,%d)", *c, x1, x2, y1, y2);
}

SCOPE void ssd_set_pixel(rtgui_color_t *c, int x, int y) {
    rt_uint8_t x1 = x & 0x000000ff;
    rt_uint8_t y1 = y & 0x000000ff;

    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_set_window(x1, x1, y1, y1);
    ssd_write_data(SSD_CTX()->ldev, (rt_uint8_t *)c, 2);
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[SSD] set pixel %04x (%d, %d)", *c, x, y);
}

SCOPE void ssd_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y) {
    rt_uint8_t y1 = y & 0x000000ff;

    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_set_window((x1 & 0x000000ff), (x2 & 0x000000ff), y1, y1);
    ssd_write_data(SSD_CTX()->ldev, pixels, (x2 - x1 + 1) * 2);
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[SSD] raw hline (%d - %d, %d)", x1, x2, y);
}

SCOPE void ssd_draw_hline(rtgui_color_t *c, int x1, int x2, int y) {
    rt_uint8_t y1 = y & 0x000000ff;

    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_draw_line(c, (x1 & 0x000000ff), (x2 & 0x000000ff), y1, y1);
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[SSD] hline %04x (%d - %d, %d)", *c, x1, x2, y);
}

SCOPE void ssd_draw_vline(rtgui_color_t *c, int x , int y1, int y2) {
    rt_uint8_t x1 = x & 0x000000ff;

    if (RT_EOK != rt_device_open(SSD_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    SSD_START();
    ssd_draw_line(c, x1, x1, (y1 & 0x000000ff), (y2 & 0x000000ff));
    SSD_STOP();
    rt_device_close(SSD_CTX()->ldev);
    LOG_D("[SSD] vline %04x (%d, %d - %d)", *c, x, y1, y2);
}

static rt_err_t bsp_ssd1331_init(rt_device_t dev) {
    struct bsp_ssd_contex *ctx = SSD_CTX();
    rt_err_t ret;
    (void)dev;

    /* open lower level device */
    ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != ret) return ret;
    SSD_START();

    do {
        rt_uint8_t *code = (rt_uint8_t *)init_code;

        /* init code*/
        while (*code) {
            ret = ssd_write_cmd(ctx->ldev, code[0], &code[2], code[1]);
            if (RT_EOK != ret) break;
            code += (2 + code[1]);
        }
        if (RT_EOK != ret) break;

        SSD_LIGHT_ON();
        LOG_D("[SSD] init ok");
    } while (0);

    SSD_STOP();
    rt_device_close(ctx->ldev);

    if (RT_EOK != ret) {
        LOG_W("[SSD E] init failed! (%08x)", ret);
    }
    return ret;
}

static rt_err_t bsp_ssd1331_control(rt_device_t dev, rt_int32_t cmd,
    void *args) {
    struct bsp_ssd_contex *ctx = SSD_CTX();
    rt_err_t ret = -RT_ERROR;
    #if !CONFIG_USING_GUI
        (void)args;
    #endif
    (void)dev;

    switch (cmd) {
    case RTGRAPHIC_CTRL_RECT_UPDATE:
        break;

    case RTGRAPHIC_CTRL_POWERON:
        ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
        if (RT_EOK != ret) break;
        SSD_START();
        ret = ssd_write_cmd(ctx->ldev, SSD_CMD_DISPLAYON, RT_NULL, 0);
        SSD_STOP();
        rt_device_close(ctx->ldev);
        if (RT_EOK != ret) break;
        SSD_LIGHT_ON();
        break;

    case RTGRAPHIC_CTRL_POWEROFF:
        ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
        if (RT_EOK != ret) break;
        SSD_START();
        ret = ssd_write_cmd(ctx->ldev, SSD_CMD_DISPLAYOFF, RT_NULL, 0);
        SSD_STOP();
        rt_device_close(ctx->ldev);
        if (RT_EOK != ret) break;
        SSD_LIGHT_OFF();
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
static rt_err_t bsp_ssd1331_contex_init(struct bsp_ssd_contex *ctx,
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
        ctx->dev.init = bsp_ssd1331_init;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = RT_NULL;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_ssd1331_control;
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
rt_err_t bsp_hw_ssd1331_init(void) {
    rt_err_t ret;

    do {
        rt_device_t ldev;

        /* get lower level device */
        ldev = rt_device_find(SSD_LOWER_DEVICE_NAME);
        if (RT_NULL == ldev) {
            LOG_D("[SSD] can't find device %s!", SSD_LOWER_DEVICE_NAME);
            ret = -RT_ERROR;
            break;
        }
        LOG_D("[SSD] find device %s", SSD_LOWER_DEVICE_NAME);

        /* switch to max speed */
        ret = rt_device_control(ldev, RT_DEVICE_CTRL_SPI_SPEED,
            (void *)SPI_MAX_SPEED);
        if (RT_EOK != ret) break;

        ret = bsp_ssd1331_contex_init(SSD_CTX(), SSD_NAME, ldev);
        if (RT_EOK != ret) break;

        SSD_IO_INIT();
        SSD_RESET();
        SSD_LIGHT_OFF();
        LOG_D("[SSD] h/w init ok");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[SSD] h/w init failed: %d", ret);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_SSD1331 */

} /* extern "C" */
