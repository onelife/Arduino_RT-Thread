/***************************************************************************//**
 * @file    drv_spiili.cpp
 * @brief   Arduino RT-Thread library ILI9341 device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_ILI
}

#include <Arduino.h>
#include <SPI.h>    /* Arduino library */

#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

extern "C" {

#include "drv_spi.h"
#include "drv_spiili.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_ILI_DEBUG
#  define LOG_LVL                   LOG_LVL_DBG
# else
#  define LOG_LVL                   LOG_LVL_INFO
# endif
# define LOG_TAG                    "ILI"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# ifdef BSP_ILI_DEBUG
#  define LOG_D(format, args...)    rt_kprintf(format "\n", ##args)
# else
#  define LOG_D(format, args...)
# endif
# define LOG_I                      LOG_D
# define LOG_HEX(format, args...)                    
#endif /* RT_USING_ULOG */

#define ILI_IO_INIT()               {   \
    pinMode(CONFIG_ILI_CS_PIN, OUTPUT); \
    pinMode(CONFIG_ILI_DC_PIN, OUTPUT); \
    digitalWrite(CONFIG_ILI_CS_PIN, HIGH); \
    digitalWrite(CONFIG_ILI_DC_PIN, HIGH); \
}
#define ILI_START()                 digitalWrite(CONFIG_ILI_CS_PIN, LOW); \
                                    LOG_D("[ILI] start")
#define ILI_STOP()                  digitalWrite(CONFIG_ILI_CS_PIN, HIGH); \
                                    LOG_D("[ILI] stop")
#define ILI_CMD_START()             digitalWrite(CONFIG_ILI_DC_PIN, LOW)
#define ILI_CMD_STOP()              digitalWrite(CONFIG_ILI_DC_PIN, HIGH)
#define _NUM_TO_STR(n)              #n
#define _CH_TO_STR(ch)              _NUM_TO_STR(ch)
#define ILI_LOWER_DEVICE_NAME       "SPI" _CH_TO_STR(CONFIG_ILI_SPI_CHANNEL)
#define ILI_CTX()                   (&ili_ctx)
#define ILI_DELAY(ms)               delay(ms)   /* Arduino function*/
#if CONFIG_USING_GUI
# define SCOPE                      static
#else
# define SCOPE
#endif

/* Private function prototypes -----------------------------------------------*/
#if CONFIG_USING_GUI
static void ili_set_pixel(rtgui_color_t *c, int x, int y);
static void ili_get_pixel(rtgui_color_t *c, int x, int y);
static void ili_draw_hline(rtgui_color_t *c, int x1, int x2, int y);
static void ili_draw_vline(rtgui_color_t *c, int x , int y1, int y2);
static void ili_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y);
#endif

/* Private constants ---------------------------------------------------------*/
static const rt_uint8_t init_code[] = {
    ILI_CMD_SWRESET, 0,
    ILI_CMD_DELAY, 0,
    0xEF, 3, 0x03, 0x80, 0x02,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00, 0x00,
    ILI_CMD_PWCTR1, 1, 0x23,                // Power control VRH[5:0]
    ILI_CMD_PWCTR2, 1, 0x10,                // Power control SAP[2:0];BT[3:0]
    ILI_CMD_VMCTR1, 2, 0x3e, 0x28,          // VCM control
    ILI_CMD_VMCTR2, 1, 0x86,                // VCM control2
    ILI_CMD_MADCTL, 1, 0x48,                // Memory Access Control: BGR?
    ILI_CMD_VSCRSADD, 1, 0x00,              // Vertical scroll zero
    ILI_CMD_PIXFMT, 1, 0x55,
    ILI_CMD_FRMCTR1, 2, 0x00, 0x18,
    ILI_CMD_DFUNCTR, 3, 0x08, 0x82, 0x27,   // Display Function Control
    0xF2, 1, 0x00,                          // 3Gamma Function Disable
    ILI_CMD_GAMMASET, 1, 0x01,              // Gamma curve selected
    ILI_CMD_GMCTRP1, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    ILI_CMD_GMCTRN1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    ILI_CMD_SLPOUT, 0,                      // Exit Sleep
    ILI_CMD_DELAY, 0,
    ILI_CMD_DISPON, 0,                      // Display on
    ILI_CMD_DELAY, 0,
    0x00                                    // End of code
};

#if CONFIG_USING_GUI
static const struct rt_device_graphic_info disp_info = {
    .pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565,
    .bits_per_pixel = 16,
    .pitch          = 0,
    .width          = CONFIG_GUI_WIDTH,
    .height         = CONFIG_GUI_HIGH,
    .framebuffer    = RT_NULL,
    .smem_len       = 0,
};

static struct rtgui_graphic_driver_ops disp_ops = {
    .set_pixel      = ili_set_pixel,
    .get_pixel      = ili_get_pixel,
    .draw_hline     = ili_draw_hline,
    .draw_vline     = ili_draw_vline,
    .draw_raw_hline = ili_draw_raw_hline,
};
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_ili_contex ili_ctx;
static rt_uint16_t ili_buf[
    (CONFIG_GUI_WIDTH > CONFIG_GUI_HIGH) ? CONFIG_GUI_WIDTH : CONFIG_GUI_HIGH];

/* Private functions ---------------------------------------------------------*/
static rt_err_t ili_write_cmd(rt_device_t ldev, rt_uint8_t cmd,
    rt_uint8_t *param, rt_uint8_t *resp, rt_uint8_t resp_len) {
    rt_uint8_t buf_ins[RT_ALIGN(ILI_MAX_PARAM_LEN + 5, RT_ALIGN_SIZE)];
    rt_err_t ret;

    if (RT_NULL != param) {
        if (param[0] > ILI_MAX_PARAM_LEN) {
            LOG_E("[ILI E] param too long");
            return -RT_ERROR;
        }
    }
    ret = RT_EOK;
    LOG_D("[ILI] send cmd %02x [%d, %d]", cmd, *param, resp_len);

    do {
        rt_size_t ret_len;

        ILI_CMD_START();
        /* send cmd */
        /* build inst
            - inst len: 1
            - inst: cmd
            - tx buf addr: NULL
         */
        buf_ins[0] = 1;
        buf_ins[1] = cmd;
        ret_len = rt_device_write(ldev, SPI_FLAG_MORE, buf_ins, 0);
        if (0 != ret_len) {
            LOG_W("[ILI E] send cmd failed!");
            ret = -RT_EIO;
            break;
        }
        ILI_CMD_STOP();

        /* send param and receive resp */
        /*  build inst
            - inst len: param[0]
            - inst: param[1:]
            - rx buf addr: offset align with RT_ALIGN_SIZE
        */
        if (RT_NULL != param) {
            rt_memcpy(buf_ins, param, param[0] + 1);
        } else {
            buf_ins[0] = 0;
        }
        if (!resp_len) {
            if (buf_ins[0]) {
                ret_len = rt_device_write(ldev, 0, buf_ins, 0);
                if (0 != ret_len) {
                    LOG_W("[ILI E] send param failed!");
                    ret = -RT_EIO;
                    break;
                }
            }
        } else {
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(rt_uint8_t **)(&buf_ins[ \
                RT_ALIGN(buf_ins[0] + 1, RT_ALIGN_SIZE)]) = resp;
            #pragma GCC diagnostic pop

            ret_len = rt_device_read(ldev, 0, buf_ins, resp_len);
            if (resp_len != ret_len) {
                LOG_E("[ILI E] get resp failed! [%d]", ret_len);
                ret = -RT_EIO;
                break;
            }
        }
    } while (0);

    return ret;
}

static rt_err_t ili_write_data(rt_device_t ldev, rt_uint8_t *data,
    rt_uint32_t len) {
    rt_uint8_t buf_ins[RT_ALIGN(5, RT_ALIGN_SIZE)];
    rt_err_t ret;

    ret = RT_EOK;
    LOG_D("[ILI] write data [%d]", len);

    do {
         rt_size_t ret_len;

         ret = ili_write_cmd(ldev, ILI_CMD_RAMWR, RT_NULL, RT_NULL, 0);
         if (RT_EOK !=ret) break;

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
            LOG_E("[ILI E] write data failed! [%d]", ret_len);
            ret = -RT_EIO;
            break;
        }
    } while (0);

    return ret;
}

static rt_err_t ili_read_data(rt_device_t ldev, rt_uint8_t *data,
    rt_uint32_t len) {
    rt_uint8_t buf_ins[RT_ALIGN(5, RT_ALIGN_SIZE)];
    rt_err_t ret;

    ret = RT_EOK;
    LOG_D("[ILI] read data [%d]", len);

    do {
         rt_size_t ret_len;

         ret = ili_write_cmd(ldev, ILI_CMD_RAMRD, RT_NULL, RT_NULL, 0);
         if (RT_EOK !=ret) break;

        /* read data
            - color in RGB666 format
            - dummy read 1 byte
            - receive RRRRRR000, GGGGGG000, BBBBBB000
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

        ret_len = rt_device_read(ldev, 0, buf_ins, len);
        if (len != ret_len) {
            LOG_E("[ILI E] read data failed! [%d]", ret_len);
            ret = -RT_EIO;
            break;
        }
        LOG_HEX("read", 16, data, len);
    } while (0);

    return ret;
}

static rt_uint8_t ili_read_reg(rt_device_t ldev, rt_uint8_t reg,
    rt_uint8_t idx) {
    rt_uint8_t buf[2];
    rt_uint8_t ret;

    buf[0] = 1;
    buf[1] = 0x10 + idx;    /* byte index */
    ili_write_cmd(ldev, 0xd9, buf, RT_NULL, 0);
    ili_write_cmd(ldev, reg, RT_NULL, &ret, 1);

    LOG_D("[ILI] reg %02x[%d] = %02x", reg, idx, ret);
    return ret;
}

static void ili_set_window(rt_uint16_t x1, rt_uint16_t x2, rt_uint16_t y1,
    rt_uint16_t y2) {
    rt_uint8_t data[5];

    if (x1 > x2) {
        rt_uint16_t t = x2;
        x2 = x1;
        x1 = t;
    }
    if (y1 > y2) {
        rt_uint16_t t = y2;
        y2 = y1;
        y1 = t;
    }
    LOG_D("[ILI] win (%d,%d) (%d,%d)", x1, y1, x2, y2);

    /* set x */
    data[0] = 4;
    data[1] = (rt_uint8_t)((x1 & 0xff00) >> 8);
    data[2] = (rt_uint8_t)(x1 & 0x00ff);
    data[3] = (rt_uint8_t)((x2 & 0xff00) >> 8);
    data[4] = (rt_uint8_t)(x2 & 0x00ff);
    (void)ili_write_cmd(ILI_CTX()->ldev, ILI_CMD_CASET, data, RT_NULL, 0);

    /* set y */
    data[0] = 4;
    data[1] = (rt_uint8_t)((y1 & 0xff00) >> 8);
    data[2] = (rt_uint8_t)(y1 & 0x00ff);
    data[3] = (rt_uint8_t)((y2 & 0xff00) >> 8);
    data[4] = (rt_uint8_t)(y2 & 0x00ff);
    (void)ili_write_cmd(ILI_CTX()->ldev, ILI_CMD_PASET, data, RT_NULL, 0);
}

SCOPE void ili_set_pixel(rtgui_color_t *c, int x, int y) {
    rt_uint16_t x1 = x & 0x0000ffff;
    rt_uint16_t y1 = y & 0x0000ffff;

    if (RT_EOK != rt_device_open(ILI_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    ILI_START();
    ili_set_window(x1, x1, y1, y1);
    ili_write_data(ILI_CTX()->ldev, (rt_uint8_t *)c, 2);
    ILI_STOP();
    rt_device_close(ILI_CTX()->ldev);
    LOG_D("[ILI] set pixel %04x (%d, %d)", *c, x, y);
}

SCOPE void ili_get_pixel(rtgui_color_t *c, int x, int y) {
    rt_uint16_t x1 = x & 0x0000ffff;
    rt_uint16_t y1 = y & 0x0000ffff;
    rt_uint8_t buf[4];

    if (RT_EOK != rt_device_open(ILI_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    ILI_START();
    ili_set_window(x1, x1, y1, y1);
    ili_read_data(ILI_CTX()->ldev, buf, 4);
    ILI_STOP();
    rt_device_close(ILI_CTX()->ldev);
    *c = (buf[1] << 16) | (buf[2] << 8) | buf[3];
    LOG_D("[ILI] get pixel %08x (%d, %d)", *c, x, y);
}

SCOPE void ili_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y) {
    rt_uint16_t y1 = y & 0x0000ffff;

    if (RT_EOK != rt_device_open(ILI_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    ILI_START();
    ili_set_window((x1 & 0x0000ffff), (x2 & 0x0000ffff), y1, y1);
    ili_write_data(ILI_CTX()->ldev, pixels, (x2 - x1 + 1) * 2);
    ILI_STOP();
    rt_device_close(ILI_CTX()->ldev);
    LOG_D("[ILI] raw hline (%d - %d, %d)", x1, x2, y);
}

SCOPE void ili_draw_hline(rtgui_color_t *c, int x1, int x2, int y) {
    rt_uint16_t y1 = y & 0x0000ffff;
    rt_uint32_t i, len;

    len = x2 - x1 + 1;
    for (i = 0; i < len; i++) ili_buf[i] = (rt_uint16_t)*c;

    if (RT_EOK != rt_device_open(ILI_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    ILI_START();
    ili_set_window((x1 & 0x0000ffff), (x2 & 0x0000ffff), y1, y1);
    ili_write_data(ILI_CTX()->ldev, (rt_uint8_t *)ili_buf, len * 2);
    ILI_STOP();
    rt_device_close(ILI_CTX()->ldev);
    LOG_D("[ILI] hline %04x (%d - %d, %d)", *c, x1, x2, y);
}

SCOPE void ili_draw_vline(rtgui_color_t *c, int x , int y1, int y2) {
    rt_uint16_t x1 = x & 0x0000ffff;
    rt_uint32_t i, len;

    len = y2 - y1 + 1;
    for (i = 0; i < len; i++) ili_buf[i] = (rt_uint16_t)*c;

    if (RT_EOK != rt_device_open(ILI_CTX()->ldev, RT_DEVICE_OFLAG_RDWR))
        return;
    ILI_START();
    ili_set_window(x1, x1, (y1 & 0x0000ffff), (y2 & 0x0000ffff));
    ili_write_data(ILI_CTX()->ldev, (rt_uint8_t *)ili_buf, len * 2);
    ILI_STOP();
    rt_device_close(ILI_CTX()->ldev);
    LOG_D("[ILI] vline %04x (%d, %d - %d)", *c, x, y1, y2);
}

static rt_err_t bsp_spiIli_init(rt_device_t dev) {
    struct bsp_ili_contex *ctx = ILI_CTX();
    rt_err_t ret;
    (void)dev;

    /* open lower level device */
    ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != ret) return ret;
    ILI_START();

    do {
        rt_uint8_t *code = (rt_uint8_t *)init_code;

        /* init code*/
        while (*code) {
            if (ILI_CMD_DELAY == code[0]) {
                ILI_DELAY(150);
            } else {
                ret = ili_write_cmd(ctx->ldev, code[0], &code[1], RT_NULL, 0);
            }
            if (RT_EOK != ret) break;
            code += (2 + code[1]);
        }
        if (RT_EOK != ret) break;

        /* read ID */
        ctx->id = ili_read_reg(ctx->ldev, ILI_CMD_ID4, 2);
        ctx->id = (ctx->id << 8) | ili_read_reg(ctx->ldev, ILI_CMD_ID4, 3);
        LOG_I("[ILI] init ok, id %04x", ctx->id);
    } while (0);

    ILI_STOP();
    rt_device_close(ctx->ldev);

    if (RT_EOK != ret) {
        LOG_W("[ILI E] init failed! (%08x)", ret);
    }
    return ret;
}

static rt_err_t bsp_spiIli_control(rt_device_t dev, rt_int32_t cmd,
    void *args) {
    rt_err_t ret = -RT_ERROR;
    #if CONFIG_USING_GUI
        struct bsp_ili_contex *ctx = ILI_CTX();
    #else
        (void)args;
    #endif
    (void)dev;

    switch (cmd) {
    case RTGRAPHIC_CTRL_RECT_UPDATE:
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
 * @brief Initialize ILI contex
 *
 * @param[in] struct bsp_ili_contex *ctx - Pointer to ILI contex
 *
 * @param[in] const char *name - Pointer to ILI name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_spiIli_contex_init(struct bsp_ili_contex *ctx,
    const char *name, rt_device_t ldev) {
    rt_err_t ret;

    do {
        ctx->id = 0;
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
        ctx->dev.init = bsp_spiIli_init;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = RT_NULL;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_spiIli_control;
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
 * @brief - initialize ILI contex and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_spiIli_init(void) {
    rt_err_t ret;

    do {
        rt_device_t ldev;

        /* get lower level device */
        ldev = rt_device_find(ILI_LOWER_DEVICE_NAME);
        if (RT_NULL == ldev) {
            LOG_D("[ILI] can't find device %s!", ILI_LOWER_DEVICE_NAME);
            ret = -RT_ERROR;
            break;
        }
        LOG_D("[ILI] find device %s", ILI_LOWER_DEVICE_NAME);

        ret = bsp_spiIli_contex_init(ILI_CTX(), ILI_NAME, ldev);
        if (RT_EOK != ret) break;

        ILI_IO_INIT();
        LOG_D("[ILI] h/w init ok");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[ILI] h/w init failed: %d", ret);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_ILI */

} /* extern "C" */
