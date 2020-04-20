/***************************************************************************//**
 * @file    drv_spi_ssd7735.cpp
 * @brief   Arduino RT-Thread library ST7735 device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_ST7735
}


#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

extern "C" {

#include "bsp/bsp.h"
#include "components/drivers/include/rtdevice.h"
#include "drv_spi_st7735.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_ST7735_DEBUG
#  define LOG_LVL                   LOG_LVL_DBG
# else
#  define LOG_LVL                   LOG_LVL_INFO
# endif
# define LOG_TAG                    "ST7735"
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

#define MIN(a, b)                   ((a < b) ? a : b)

#define DISP_START()                gpio_bit_reset(CONFIG_ST7735_PORT, CONFIG_ST7735_CS_PIN)
#define DISP_STOP()                 gpio_bit_set(CONFIG_ST7735_PORT, CONFIG_ST7735_CS_PIN)
#define CMD_START()                 gpio_bit_reset(CONFIG_ST7735_PORT, CONFIG_ST7735_DC_PIN)
#define CMD_STOP()                  gpio_bit_set(CONFIG_ST7735_PORT, CONFIG_ST7735_DC_PIN)

#define _TO_STR(n)                  #n
#define _CH_TO_STR(ch)              _TO_STR(ch)
#define SPI_BUS_NAME                "SPI" _CH_TO_STR(CONFIG_ST7735_SPI_CHANNEL)
#define DISP_X_OFFSET               (1)
#define DISP_Y_OFFSET               (26)
#define DISP_CTX()                  (&disp_ctx)
#if CONFIG_USING_GUI
# define SCOPE                      static
#else
# define SCOPE
#endif

/* Private function prototypes -----------------------------------------------*/
#if CONFIG_USING_GUI
static void disp_set_pixel(rtgui_color_t *c, int x, int y);
static void disp_get_pixel(rtgui_color_t *c, int x, int y);
static void disp_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y);
static void disp_draw_hline(rtgui_color_t *c, int x1, int x2, int y);
static void disp_draw_vline(rtgui_color_t *c, int x , int y1, int y2);
#endif

/* Private constants ---------------------------------------------------------*/
static const rt_uint8_t init_code[] = {
    ST7735_CMD_SLPOUT,  0,
    ST7735_CMD_DELAY,   10,
    ST7735_CMD_INVON,   0,
    // normal mode / full colors frame frequency
    // Frame rate=fosc/((RTNA x 2 + 40) x (LINE + FPA + BPA +2))
    // fosc = 850kHz
    ST7735_CMD_FRMCTR1, 3, 0x05, 0x3A, 0x3A,    // RTNA, FPA, BPA
    // idle mode / 8-color frame frequency
    // Frame rate=fosc/((RTNB x 2 + 40) x (LINE + FPB + BPB +2))
    ST7735_CMD_FRMCTR2, 3, 0x05, 0x3A, 0x3A,    // RTNB, FPA, BPA
    // partial mode / full colors frame frequency
    ST7735_CMD_FRMCTR3, 6, 0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A,
    ST7735_CMD_INVCTR,  1, 0x03,
    ST7735_CMD_PWCTR1,  3, 0x62, 0x02, 0x04,
    ST7735_CMD_PWCTR2,  1, 0xC0,
    ST7735_CMD_PWCTR3,  2, 0x0D, 0x00,
    ST7735_CMD_PWCTR4,  2, 0x8D, 0x6A,
    ST7735_CMD_PWCTR5,  2, 0x8D, 0xEE,
    ST7735_CMD_VMCTR1,  1, 0x0E,
    ST7735_CMD_GMCTRP1,16, 0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07,
                           0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10,
    ST7735_CMD_GMCTRN1,16, 0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08,
                           0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10,
    ST7735_CMD_COLMOD,  1, 0x05,            // 16-bit color
    ST7735_CMD_MADCTL,  1, 0x78,
    ST7735_CMD_DISPON,  0,
    0x00
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
    .set_pixel      = disp_set_pixel,
    .get_pixel      = disp_get_pixel,
    .draw_hline     = disp_draw_hline,
    .draw_vline     = disp_draw_vline,
    .draw_raw_hline = disp_draw_raw_hline,
};
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_disp_contex disp_ctx;
static struct rt_spi_device disp_ldev;
static rt_uint16_t disp_buf[
    ((CONFIG_GUI_WIDTH > CONFIG_GUI_HIGH) ? CONFIG_GUI_WIDTH : CONFIG_GUI_HIGH)];

/* Private functions ---------------------------------------------------------*/
static void delay_ticks(rt_tick_t cnt) {
    rt_tick_t tick_now = rt_tick_get();
    rt_tick_t tick_end = tick_now + cnt;
    
    while (tick_end > tick_now) {
        tick_now = rt_tick_get();
    }
}

static rt_err_t disp_take_bus(struct bsp_disp_contex *ctx) {
    rt_err_t ret;

    LOG_D("[7735] take bus");
    ret = rt_mutex_take(&(ctx->ldev->bus->lock), RT_WAITING_FOREVER);
    if ((RT_EOK == ret) && (ctx->ldev != ctx->ldev->bus->owner)) {
        /* re-config */
        ret = ctx->ldev->bus->ops->configure(ctx->ldev, &ctx->ldev->config);
        if (RT_EOK == ret) {
            ctx->ldev->bus->owner = ctx->ldev;
        }
    }
    return ret;
}

static rt_err_t disp_release_bus(struct bsp_disp_contex *ctx) {
    LOG_D("[7735] release bus");
    return rt_mutex_release(&(ctx->ldev->bus->lock));
}

static rt_err_t disp_write_reg(struct bsp_disp_contex *ctx, rt_uint8_t reg,
    rt_uint8_t *param, rt_size_t len) {
    struct rt_spi_message msg;
    rt_err_t ret;

    if ((len > 0) && (param == NULL))
        return RT_EINVAL;
    LOG_D("[7735] write reg %02x (%d)", reg, len);

    DISP_START();

    do {
        /* send cmd */
        msg.send_buf = (const void *)&reg;
        msg.recv_buf = NULL;
        msg.length = 1;
        msg.next = NULL;
        msg.cs_take = 0;
        msg.cs_release = 0;
        CMD_START();
        (void)rt_spi_transfer_message(ctx->ldev, &msg);
        CMD_STOP();
        if (RT_EOK != (ret = rt_get_errno()))
            break;
        if (len != 0) {
            /* send param */
            msg.send_buf = (const void *)param;
            msg.recv_buf = NULL;
            msg.length = len;
            msg.next = NULL;
            msg.cs_take = 0;
            msg.cs_release = 0;
            (void)rt_spi_transfer_message(ctx->ldev, &msg);
            if (RT_EOK != (ret = rt_get_errno()))
                break;
        }
    } while (0);

    DISP_STOP();

    if (RT_EOK != ret) {
        LOG_W("[7735] write err %d", ret);
    }
    return ret;
}

static rt_err_t disp_read_reg(struct bsp_disp_contex *ctx, rt_uint8_t reg,
    rt_uint8_t *buf, rt_size_t len) {
    rt_err_t ret;
    struct rt_spi_message msg;

    if ((len == 0) || (buf == NULL)) {
        return RT_EINVAL;
    }
    LOG_D("[7735] read reg %02x", reg);

    DISP_START();

    do {
        /* send cmd */
        msg.send_buf = (const void *)&reg;
        msg.recv_buf = NULL;
        msg.length = 1;
        msg.next = NULL;
        msg.cs_take = 0;
        msg.cs_release = 0;
        CMD_START();
        (void)rt_spi_transfer_message(ctx->ldev, &msg);
        CMD_STOP();
        if (RT_EOK != (ret = rt_get_errno()))
            break;
        /* get data */
        msg.send_buf = NULL;
        msg.recv_buf = (void *)buf;
        msg.length = len;
        (void)rt_spi_transfer_message(ctx->ldev, &msg);
        if (RT_EOK != (ret = rt_get_errno()))
            break;
    } while (0);

    DISP_STOP();

    if (RT_EOK != ret) {
        LOG_W("[7735] read err %d", ret);
    }
    return ret;
}

static void disp_set_window(struct bsp_disp_contex *ctx, rt_uint16_t x1,
    rt_uint16_t x2, rt_uint16_t y1, rt_uint16_t y2) {
    rt_uint8_t data[4];

    /* set x */
    if (x1 > x2) {
        data[0] = 0;
        data[1] = (x2 + DISP_X_OFFSET) & 0x00ff;
        data[2] = 0;
        data[3] = (x1 + DISP_X_OFFSET) & 0x00ff;
    } else {
        data[0] = 0;
        data[1] = (x1 + DISP_X_OFFSET) & 0x00ff;
        data[2] = 0;
        data[3] = (x2 + DISP_X_OFFSET) & 0x00ff;
    }
    (void)disp_write_reg(ctx, ST7735_CMD_CASET, data, sizeof(data));

    /* set y */
    if (y1 > y2) {
        data[0] = 0;
        data[1] = (y2 + DISP_Y_OFFSET) & 0x00ff;
        data[2] = 0;
        data[3] = (y1 + DISP_Y_OFFSET) & 0x00ff;
    } else {
        data[0] = 0;
        data[1] = (y1 + DISP_Y_OFFSET) & 0x00ff;
        data[2] = 0;
        data[3] = (y2 + DISP_Y_OFFSET) & 0x00ff;
    }
    (void)disp_write_reg(ctx, ST7735_CMD_RASET, data, sizeof(data));

    LOG_D("[7735] win (%d,%d) (%d,%d)", x1, y1, x2, y2);
}

SCOPE void disp_set_pixel(rtgui_color_t *c, int x, int y) {
    struct bsp_disp_contex *ctx = DISP_CTX();
    rt_uint16_t x1 = x & 0x000000ff;
    rt_uint16_t y1 = y & 0x000000ff;

    if (RT_EOK != disp_take_bus(ctx)) return;
    disp_set_window(ctx, x1, x1, y1, y1);
    (void)disp_write_reg(ctx, ST7735_CMD_RAMWR, (rt_uint8_t *)c, 2);
    (void)disp_release_bus(ctx);
    LOG_D("[7735] set pixel %04x (%d, %d)", *c, x, y);
}

SCOPE void disp_get_pixel(rtgui_color_t *c, int x, int y) {
    struct bsp_disp_contex *ctx = DISP_CTX();
    rt_uint16_t x1 = x & 0x000000ff;
    rt_uint16_t y1 = y & 0x000000ff;

    if (RT_EOK != disp_take_bus(ctx)) return;
    disp_set_window(ctx, x1, x1, y1, y1);
    (void)disp_read_reg(ctx, ST7735_CMD_RAMRD, (rt_uint8_t *)c, 2);
    (void)disp_release_bus(ctx);
    LOG_D("[7735] get pixel %04x (%d, %d)", *c, x, y);
}

SCOPE void disp_draw_raw_hline(rt_uint8_t *pixels, int x1, int x2, int y) {
    struct bsp_disp_contex *ctx = DISP_CTX();
    rt_uint16_t y1 = y & 0x000000ff;
    rt_size_t len = MIN((x2 - x1 + 1), CONFIG_GUI_WIDTH);

    if (RT_EOK != disp_take_bus(ctx)) return;
    disp_set_window(ctx, (x1 & 0x000000ff), (x2 & 0x000000ff), y1, y1);
    (void)disp_write_reg(ctx, ST7735_CMD_RAMWR, pixels, len * 2);
    (void)disp_release_bus(ctx);
    LOG_D("[7735] raw hline (%d - %d, %d)", x1, x2, y);
}

SCOPE void disp_draw_hline(rtgui_color_t *c, int x1, int x2, int y) {
    struct bsp_disp_contex *ctx = DISP_CTX();
    rt_uint16_t y1 = y & 0x000000ff;
    rt_size_t i, len;

    len = MIN((x2 - x1 + 1), CONFIG_GUI_WIDTH);
    for (i = 0; i < len; i++)
        disp_buf[i] = (rt_uint16_t)*c;

    if (RT_EOK != disp_take_bus(ctx))
        return;
    disp_set_window(ctx, (x1 & 0x000000ff), (x2 & 0x000000ff), y1, y1);
    (void)disp_write_reg(ctx, ST7735_CMD_RAMWR, (rt_uint8_t *)disp_buf, len * 2);
    (void)disp_release_bus(ctx);
    LOG_D("[7735] hline %04x (%d - %d, %d)", *c, x1, x2, y);
}

SCOPE void disp_draw_vline(rtgui_color_t *c, int x , int y1, int y2) {
    struct bsp_disp_contex *ctx = DISP_CTX();
    rt_uint8_t x1 = x & 0x000000ff;
    rt_size_t i, len;
    
    len = MIN((y2 - y1 + 1), CONFIG_GUI_HIGH);
    for (i = 0; i < len; i++)
        disp_buf[i] = (rt_uint16_t)*c;

    if (RT_EOK != disp_take_bus(ctx))
        return;
    disp_set_window(ctx, x1, x1, (y1 & 0x000000ff), (y2 & 0x000000ff));
    (void)disp_write_reg(ctx, ST7735_CMD_RAMWR, (rt_uint8_t *)disp_buf, len * 2);
    (void)disp_release_bus(ctx);
    LOG_D("[7735] vline %04x (%d, %d - %d)", *c, x, y1, y2);
}

static rt_err_t bsp_disp_init(rt_device_t dev) {
    struct bsp_disp_contex *ctx = DISP_CTX();
    rt_bool_t got_bus = RT_FALSE;
    rt_err_t ret;
    (void)dev;

    do {
        struct rt_spi_configuration cfg = {
            .mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_3WIRE,
            .data_width = 8,
            .reserved = 0,
            .max_hz = 1000 * 1000 * 14,
        };
        const rt_uint8_t *code = init_code;

        ret = rt_spi_configure(ctx->ldev, &cfg);
        if (RT_EOK != ret) break;

        ret = disp_take_bus(ctx);
        if (RT_EOK != ret) break;
        got_bus = RT_TRUE;

        /* init code*/
        while (*code) {
            if (ST7735_CMD_DELAY == code[0]) {
                delay_ticks(code[1]);
                code += 2;
            } else {
                if (RT_EOK != (ret = disp_write_reg(ctx, code[0],
                    (rt_uint8_t *)&code[2], code[1]))) {
                    break;
                }
                code += (2 + code[1]);
            }
        }
        if (RT_EOK != ret) break;
        LOG_D("init ok");

        /* read ID */
        if (RT_EOK != (ret = disp_read_reg(ctx, ST7735_CMD_RDDID,
            (rt_uint8_t *)&ctx->id, sizeof(ctx->id)))) {
            break;
        }
        ctx->id >>= 8;
        LOG_I("[7735] init ok, id 0x%04x", ctx->id);
    } while (0);

    if (got_bus) {
        disp_release_bus(ctx);
    }
    if (RT_EOK != ret) {
        LOG_W("[7735 E] init failed! (%08x)", ret);
    }
    return ret;
}

static rt_err_t bsp_disp_control(rt_device_t dev, rt_int32_t cmd,
    void *args) {
    rt_err_t ret = -RT_ERROR;
    #if !CONFIG_USING_GUI
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
        {
            struct bsp_disp_contex *ctx = DISP_CTX();
            rt_memcpy(args, ctx->disp_info, sizeof(struct rt_device_graphic_info));
            ret = RT_EOK;
        }
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
 * @brief Initialize display context
 *
 * @param[in] struct bsp_disp_contex *ctx - Pointer to ST7735 context
 *
 * @param[in] const char *name - Pointer to ST7735 name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t disp_contex_init(struct bsp_disp_contex *ctx,
    const char *name, struct rt_spi_device *ldev) {
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
        ctx->dev.init = bsp_disp_init;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = RT_NULL;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_disp_control;
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
 * @brief - initialize ST7735 context and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_st7735_init(void) {
    rt_err_t ret;

    do {
        /* register SPI device */
        ret = rt_spi_bus_attach_device(&disp_ldev, "SPI" ST7735_NAME,
            SPI_BUS_NAME, NULL);
        if (RT_EOK != ret) {
            LOG_D("[7735] register %s err!", "SPI" ST7735_NAME);
            break;
        }
        LOG_D("[7735] register %s", "SPI" ST7735_NAME);

        ret = disp_contex_init(DISP_CTX(), ST7735_NAME, &disp_ldev);
        if (RT_EOK != ret) break;

        /* enable clock */
        rcu_periph_clock_enable(CONFIG_ST7735_PORT_CLK);
        /* setup GPIO */
        gpio_init(CONFIG_ST7735_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
            CONFIG_ST7735_CS_PIN | CONFIG_ST7735_DC_PIN | \
            CONFIG_ST7735_RST_PIN);

        /* reset */
        gpio_bit_set(CONFIG_ST7735_PORT, CONFIG_ST7735_CS_PIN);
        gpio_bit_set(CONFIG_ST7735_PORT, CONFIG_ST7735_DC_PIN);
        gpio_bit_reset(CONFIG_ST7735_PORT, CONFIG_ST7735_RST_PIN);
        delay_ticks(RT_TICK_PER_SECOND / 100 * 20);
        gpio_bit_set(CONFIG_ST7735_PORT, CONFIG_ST7735_RST_PIN);
        delay_ticks(RT_TICK_PER_SECOND / 100 * 20);
        LOG_D("[7735] h/w init ok");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[7735] h/w init failed: %d", ret);
    }
    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_ST7735 */

} /* extern "C" */
