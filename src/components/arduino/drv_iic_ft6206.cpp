/***************************************************************************//**
 * @file    drv_iic_ft6202.cpp
 * @brief   Arduino RT-Thread library FT6202 device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_FT6206
}

#include <Arduino.h>
#include <Wire.h>   /* Arduino library */

#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

extern "C" {

#include "drv_iic.h"
#include "drv_iic_ft6206.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
typedef void (*rx_indicator)(void);

/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_FT6206_DEBUG
#  define LOG_LVL                       LOG_LVL_DBG
# else
#  define LOG_LVL                       LOG_LVL_INFO
# endif
# define LOG_TAG                        "FT6206"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)         rt_kprintf(format "\n", ##args)
# define LOG_W                          LOG_E
# ifdef BSP_FT6206_DEBUG
#  define LOG_D(format, args...)        rt_kprintf(format "\n", ##args)
# else
#  define LOG_D(format, args...)
# endif
# define LOG_I                          LOG_D
# define LOG_HEX(format, args...)                    
#endif /* RT_USING_ULOG */

#ifdef CONFIG_FT6206_INT_PIN
# define FT_IO_INIT() {                 \
    pinMode(CONFIG_FT6206_INT_PIN, INPUT); \
    attachInterrupt(digitalPinToInterrupt(CONFIG_FT6206_INT_PIN), ft_isr, FALLING); \
}
#else
# define FT_IO_INIT()
#endif

#define _NUM_TO_STR(n)                  #n
#define _CH_TO_STR(ch)                  _NUM_TO_STR(ch)
#define FT_LOWER_DEVICE_NAME            "IIC" _CH_TO_STR(CONFIG_FT6206_IIC_CHANNEL)
#define FT_CTX()                        (&ft_ctx)

/* Private function prototypes -----------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct bsp_ft_contex ft_ctx;
static rx_indicator touch_indicator = RT_NULL;

/* Private functions ---------------------------------------------------------*/
static void ft_isr(void) {
    if (touch_indicator) touch_indicator();
}

static rt_uint8_t ft_read_reg(struct bsp_ft_contex *ctx, rt_uint8_t reg) {
    rt_off_t addr;
    rt_uint8_t ret[1];

    addr = IIC_FLAG_SLAVE_ADDR(FT6206_ADDR) | IIC_FLAG_ACCESS_ADDR(reg);

    if (1 != rt_device_read(ctx->ldev, addr, ret, 1)) {
        LOG_E("[FT E] read reg err");
        return 0xFF;
    }

    LOG_D("[FT] reg %02x = %02x", reg, ret[0]);
    return ret[0];
}

static void ft_write_reg(struct bsp_ft_contex *ctx, rt_uint8_t reg,
    rt_uint8_t data) {
    rt_off_t addr;

    addr = IIC_FLAG_SLAVE_ADDR(FT6206_ADDR) | IIC_FLAG_ACCESS_ADDR(reg);

    if (1 != rt_device_write(ctx->ldev, addr, &data, 1)) {
        LOG_E("[FT E] write reg err");
    }
    return;
}

static rt_err_t ft_read_data(struct bsp_ft_contex *ctx) {
    rt_off_t addr;
    rt_uint8_t data[14];
    rt_uint16_t x, y;
    rt_uint8_t i;

    addr = IIC_FLAG_SLAVE_ADDR(FT6206_ADDR) | IIC_FLAG_ACCESS_ADDR(0x01);

    if (14 != rt_device_read(ctx->ldev, addr, data, 14)) {
        LOG_E("[FT E] read data err");
        ctx->gesture = 0;
        ctx->touch_num = 0;
        return -RT_EIO;
    }
    LOG_HEX("ft_data", 16, data, sizeof(data));

    /* update*/
    ctx->gesture = data[0];
    LOG_D("[FT] gesture %x", data[0]);

    ctx->touch_num = data[1] & 0x0f;
    LOG_D("[FT] touch_num %d", data[1] & 0x0f);

    for (i = 0; i < FT6206_TOUCH_NUM; i++) {
        x = ((data[2 + i * 6] & 0x0f) << 8) | data[3 + i * 6];
        y = ((data[4 + i * 6] & 0x0f) << 8) | data[5 + i * 6];

        #if CONFIG_USING_GUI
            ctx->touchs[i].id = (data[4 + i * 6] & 0xf0) >> 4;
            switch (data[2 + i * 6] & 0xc0) {
            case (0 << 6):  ctx->touchs[i].type = RTGUI_TOUCH_DOWN;     break;
            case (1 << 6):  ctx->touchs[i].type = RTGUI_TOUCH_UP;       break;
            case (2 << 6):  ctx->touchs[i].type = RTGUI_TOUCH_MOTION;   break;
            default:        ctx->touchs[i].type = RTGUI_TOUCH_NONE;     break;
            }
            ctx->touchs[i].point.x = \
                (CONFIG_GUI_WIDTH > x) ? CONFIG_GUI_WIDTH - x : 0;
            ctx->touchs[i].point.y = \
                (CONFIG_GUI_HIGH > y) ? CONFIG_GUI_HIGH - y : 0;
        #endif
        LOG_D("[FT] id%d %x", i, (data[4 + i * 6] & 0xf0) >> 4);
        LOG_D("[FT] event%d %x", i, (data[2 + i * 6] & 0xc0) >> 6);
        LOG_D("[FT] point%d (%d,%d)", i,
            (CONFIG_GUI_WIDTH > x) ? CONFIG_GUI_WIDTH - x : 0,
            (CONFIG_GUI_HIGH > y) ? CONFIG_GUI_HIGH - y : 0);
    }
    (void)x;
    (void)y;

    return RT_EOK;
}

static rt_err_t bsp_ft6206_init(rt_device_t dev) {
    struct bsp_ft_contex *ctx = (struct bsp_ft_contex *)(dev->user_data);
    rt_err_t ret;

    /* open lower level device */
    ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != ret) return ret;

    do {
        rt_uint8_t id;

        ft_read_data(ctx);      /* dummy read */
        id = ft_read_reg(ctx, FT6206_REG_VENDID);
        ctx->id = id << 8;
        id = ft_read_reg(ctx, FT6206_REG_CHIPID);
        ctx->id |= id;
        // id = ft_read_reg(ctx, FT6206_REG_FIRMID);
        // LOG_D("[FT] firmid %x", id);

        ft_write_reg(ctx, FT6206_REG_THRESHHOLD, DEFAULT_THRESHOLD);
        id = ft_read_reg(ctx, FT6206_REG_THRESHHOLD);
        LOG_D("[FT] set TH %d", id);

        // ft_write_reg(ctx, FT6206_REG_G_MODE, DEFAULT_INTERRUPT_MODE);
        // id = ft_read_reg(ctx, FT6206_REG_G_MODE);
        // LOG_D("[FT] set mode %d", id);

        if (ctx->id != ((FT6206_VENDID << 8) | FT6206_CHIPID)) {
            LOG_E("[FT] bad id %04x", ctx->id);
            ret = -RT_ERROR;
            break;
        }

        LOG_I("[FT] init ok, id %04x", ctx->id);
    } while (0);

    rt_device_close(ctx->ldev);

    if (RT_EOK != ret) {
        LOG_W("[FT E] init failed! (%08x)", ret);
    }
    return ret;
}

static rt_size_t bsp_ft6206_read(rt_device_t dev, rt_off_t pos, void *buf,
    rt_size_t size) {
    struct bsp_ft_contex *ctx = (struct bsp_ft_contex *)(dev->user_data);
    rt_err_t err;
    #if CONFIG_USING_GUI
        rtgui_touch_t **ptr = (rtgui_touch_t **)buf;
    #else
        (void)buf;
    #endif
    (void)pos;
    (void)size;

    do {
        err = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
        if (RT_EOK != err) break;

        /* return touchs */
        #if CONFIG_USING_GUI
            err = ft_read_data(ctx);
            if (RT_EOK != err) {
                rt_device_close(ctx->ldev);
                break;
            }
            *ptr = ctx->touchs;
        #else
            err = RT_EOK;
        #endif

        rt_device_close(ctx->ldev);
    } while (0);

    if (RT_EOK != err) {
        rt_set_errno(err);
        return 0;
    }

    LOG_D("[FT] read %d", ctx->touch_num);
    return ctx->touch_num;
}

static rt_err_t bsp_ft6206_control(rt_device_t dev, rt_int32_t cmd, void *args) {
    rt_err_t ret = -RT_ERROR;
    (void)dev;

    switch (cmd) {
    case RT_DEVICE_CTRL_SET_RX_INDICATOR:
        touch_indicator = (rx_indicator)args;
        ret = RT_EOK;
        break;

    default:
        break;
    }

    return ret;
}

/***************************************************************************//**
 * @brief Initialize FT6206 contex
 *
 * @param[in] struct bsp_ft_contex *ctx - Pointer to FT6206 contex
 *
 * @param[in] const char *name - Pointer to FT6206 device name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_ft6206_contex_init(struct bsp_ft_contex *ctx,
    const char *name, rt_device_t ldev) {
    rt_err_t ret;

    do {
        rt_memset(ctx, 0x00, sizeof(struct bsp_ft_contex));
        ctx->ldev = ldev;

        /* register device */
        ctx->dev.type = RT_Device_Class_Miscellaneous;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = bsp_ft6206_init;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = bsp_ft6206_read;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_ft6206_control;
        ctx->dev.user_data = (void *)ctx;
        ret = rt_device_register(&ctx->dev, name,
            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
    } while (0);

    return ret;
}

/* Public functions ----------------------------------------------------------*/
/***************************************************************************//**
 * @brief - initialize TFT6206 contex and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_ft6206_init(void) {
    rt_err_t ret;

    do {
        rt_device_t ldev;

        /* get lower level device */
        ldev = rt_device_find(FT_LOWER_DEVICE_NAME);
        if (RT_NULL == ldev) {
            LOG_E("[FT] can't find device %s!", FT_LOWER_DEVICE_NAME);
            ret = -RT_ERROR;
            break;
        }
        LOG_D("[FT] find device %s", FT_LOWER_DEVICE_NAME);

        ret = bsp_ft6206_contex_init(FT_CTX(), FT6206_NAME, ldev);
        if (RT_EOK != ret) break;

        touch_indicator = RT_NULL;
        FT_IO_INIT();
        LOG_D("[FT] h/w init ok");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[FT E] h/w init failed: %d", ret);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_FT6206 */

} /* extern "C" */
