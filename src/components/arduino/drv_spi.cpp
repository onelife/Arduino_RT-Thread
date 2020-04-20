/***************************************************************************//**
 * @file    drv_spi.cpp
 * @brief   Arduino RT-Thread library SPI device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && !CONFIG_USING_DRIVER_SPI && (CONFIG_USING_SPI0 || CONFIG_USING_SPI1)
}

#include <Arduino.h>
#include <SPI.h>    /* Arduino library */

extern "C" {

#include "drv_spi.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_SPI_DEBUG
#  define LOG_LVL                   LOG_LVL_DBG
# else
#  define LOG_LVL                   LOG_LVL_INFO
# endif
# define LOG_TAG                    "SPI"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# ifdef BSP_SPI_DEBUG
#  define LOG_I(format, args...)    rt_kprintf(format "\n", ##args)
# else
#  define LOG_I(format, args...)
# endif
# define LOG_D                      LOG_I
# define LOG_HEX(format, args...)
#endif /* RT_USING_ULOG */

#if CONFIG_USING_SPI0
# define _SPI0                      SPI
#endif
#if CONFIG_USING_SPI1
# define _SPI1                      SPI1
#endif
#define SPI_NAME(ch)                "SPI"#ch
#define SPI_CTX(idx)                spi_ctx[idx]
#define SPI_DEV(ctx)                ((SPIClass *)((ctx)->ldev))
#define SPI_START(ctx)              { \
    if (!ctx->start) { \
        LOG_D("[SPI%d] START", ctx->chn); \
        SPI_DEV(ctx)->beginTransaction(ctx->set); \
        ctx->start = RT_TRUE; \
    } \
}
#define SPI_STOP(ctx)               { \
    if (ctx->start) { \
        LOG_D("[SPI%d] STOP", ctx->chn); \
        SPI_DEV(ctx)->endTransaction(); \
        ctx->start = RT_FALSE; \
    } \
}
#define SPI_TX(ctx, data)           (void)SPI_DEV(ctx)->transfer((rt_uint8_t)data)
#define SPI_RX(ctx)                 SPI_DEV(ctx)->transfer(0xff)
#define SPI_SET_SPEED(ctx, target)  \
if (target != ctx->spd) { \
    ctx->set = SPISettings(target, MSBFIRST, SPI_MODE0); \
    ctx->spd = target; \
    LOG_D("[SPI%d] speed=%d", ctx->chn, target); \
}

#define WAIT_IDLE(flags)            (!!(flags & 0x000000ff))
#define WAIT_READ(flags)            (!!(flags & 0x0000ff00))
#define IS_PENDING(flags)           (!!(flags & SPI_FLAG_MORE))
#define IDLE_TOKEN(flags)           (rt_uint8_t)(flags & 0x000000ff)
#define READ_TOKEN(flags)           (rt_uint8_t)((flags & 0x0000ff00) >> 8)

/* Private variables ---------------------------------------------------------*/
static struct bsp_spi_contex spi_ctx[SPI_CH_NUM];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static rt_bool_t wait_token(struct bsp_spi_contex *ctx, rt_uint8_t token) {
    rt_uint32_t i;

    for (i = 0; i < SPI_DEFAULT_LIMIT; i++)
        if (token == SPI_RX(ctx)) break;

    LOG_D("[SPI%d] wait token %d", ctx->chn, i);
    return (i < SPI_DEFAULT_LIMIT);
}

static rt_err_t bsp_spi_open(rt_device_t dev, rt_uint16_t oflag) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != ret) break;

        ctx->dev.open_flag = oflag & RT_DEVICE_OFLAG_MASK;
        LOG_D("[SPI%d] open with flag %x", ctx->chn, oflag);
    } while (0);

    if (RT_EOK != ret) {
        LOG_W("[SPI%d E] open failed [%08x]", ctx->chn, ret);
    }
    return ret;
}

static rt_err_t bsp_spi_close(rt_device_t dev) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_release(&ctx->lok);
        if (RT_EOK != ret) break;

        LOG_D("[SPI%d] closed", ctx->chn);
    } while (0);

    if (RT_EOK != ret) {
        LOG_W("[SPI%d E] close failed [%08x]", ctx->chn, ret);
    }
    return ret;
}

static rt_size_t bsp_spi_read(rt_device_t dev, rt_off_t flags, void *buf,
    rt_size_t size) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_bool_t locked;
    rt_err_t err;
    rt_size_t ret;

    do {
        rt_size_t i;

        locked = RT_FALSE;
        ret = 0;

        if (RT_DEVICE_OFLAG_WRONLY == (ctx->dev.open_flag & 0x0003)) {
            err = -RT_EINVAL;
            break;
        }

        err = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != err) break;
        locked = RT_TRUE;

        if (RT_NULL == buf) {
            /* dummy read */
            LOG_D("[SPI%d] dummy read [%d]", ctx->chn, size);
            SPI_START(ctx);
            for (i = 0; i < size; i++) (void)SPI_RX(ctx);
            SPI_STOP(ctx);

            ret = size;
        } else {
            /*  [operation]
                - tx inst 
                - rx data
                [data format]
                - inst len: 1-byte
                - inst: n-byte
                - rx buf addr: offset align with RT_ALIGN_SIZE
            */
            rt_uint8_t inst_len = *(rt_uint8_t *)buf;
            rt_uint8_t *inst_ptr = (rt_uint8_t *)buf + 1;
            rt_uint8_t *rx_buf = *((rt_uint8_t **)((rt_uint8_t *)buf + \
                                 RT_ALIGN(1 + inst_len, RT_ALIGN_SIZE)));

            /* wait for idle */
            SPI_START(ctx);
            if (WAIT_IDLE(flags)) {
                for (i = 0; i < SPI_DEFAULT_RETRY; i++)
                    if (wait_token(ctx, IDLE_TOKEN(flags))) break;
                if (i >= SPI_DEFAULT_RETRY) {
                    SPI_STOP(ctx);
                    err = -RT_EBUSY;
                    LOG_W("[SPI%d E] read busy", ctx->chn);
                    break;
                }
            }

            /* send instruction */
            LOG_D("[SPI%d] tx ins [%d]", ctx->chn, inst_len);
            for (i = 0; i < inst_len; i++)
                SPI_TX(ctx, *(inst_ptr + i));

            /* receive data */
            LOG_D("[SPI%d] rx data [%d]", ctx->chn, size);
            if (WAIT_READ(flags)) {
                for (i = 0; i < SPI_DEFAULT_RETRY; i++)
                    if (wait_token(ctx, READ_TOKEN(flags))) break;
                if (i >= SPI_DEFAULT_RETRY) {
                    SPI_STOP(ctx);
                    err = -RT_EIO;
                    LOG_W("[SPI%d E] no token", ctx->chn);
                    break;
                }
            }
            for (i = 0; i < size; i++)
                *(rx_buf++) = SPI_RX(ctx);
            if (!IS_PENDING(flags)) SPI_STOP(ctx);

            ret = size;
        }
    } while (0);

    if (locked) rt_mutex_release(&ctx->lok);
    if (RT_EOK != err) rt_set_errno(err);

    return ret;
}

static rt_size_t bsp_spi_write(rt_device_t dev, rt_off_t flags, const void *buf,
    rt_size_t size) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_bool_t locked;
    rt_err_t err;
    rt_size_t ret;

    if (RT_NULL == buf) return -RT_EINVAL;
    if (RT_DEVICE_OFLAG_RDONLY == (ctx->dev.open_flag & 0x0003))
        return -RT_EINVAL;

    do {
        /*  [operation]
            - tx inst 
            - tx data
            [data format]
            - inst len: 1-byte
            - inst: n-byte
            - tx buf addr: offset align with RT_ALIGN_SIZE
        */
        rt_uint8_t inst_len = *(rt_uint8_t *)buf;
        rt_uint8_t *inst_ptr = (rt_uint8_t *)buf + 1;
        rt_uint8_t *tx_buf = *((rt_uint8_t **)((rt_uint8_t *)buf + \
                             RT_ALIGN(1 + inst_len, RT_ALIGN_SIZE)));
        rt_size_t i;

        locked = RT_FALSE;
        ret = 0;

        err = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != err) break;
        locked = RT_TRUE;

        /* busy wait */
        SPI_START(ctx);
        if (WAIT_IDLE(flags)) {
            for (i = 0; i < SPI_DEFAULT_RETRY; i++)
                if (wait_token(ctx, (IDLE_TOKEN(flags)))) break;
            if (i >= SPI_DEFAULT_RETRY) {
                SPI_STOP(ctx);
                err = -RT_EBUSY;
                LOG_W("[SPI%d E] write busy", ctx->chn);
                break;
            }
        }

        /* send instruction */
        LOG_D("[SPI%d] tx ins [%d]", ctx->chn, inst_len);
        for (i = 0; i < inst_len; i++)
            SPI_TX(ctx, *(inst_ptr + i));

        /* send data */
        LOG_D("[SPI%d] tx data [%d]", ctx->chn, size);
        for (i = 0; i < size; i++)
            SPI_TX(ctx, *(tx_buf + i));
        if (!IS_PENDING(flags)) SPI_STOP(ctx);

        ret = size;
    } while (0);

    if (locked) rt_mutex_release(&ctx->lok);
    if (RT_EOK != err) rt_set_errno(err);

    return ret;
}

static rt_err_t bsp_spi_control(rt_device_t dev, rt_int32_t cmd, void *args) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_err_t ret;

    /* lock */
    ret = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
    if (RT_EOK != ret) return ret;

    do {
        /* process cmd */
        switch (cmd) {
        case RT_DEVICE_CTRL_SPI_SPEED:
            SPI_SET_SPEED(ctx, (rt_uint32_t)args);
            ret = RT_EOK;
            break;

        default:
            ret = -RT_EINVAL;
            break;
        }
        /* unlock */
        rt_mutex_release(&ctx->lok);
    } while (0);

    return ret;
}

/***************************************************************************//**
 * @brief Initialize SPI contex
 *
 * @param[in] struct bsp_spi_contex *ctx - Pointer to SPI contex
 *
 * @param[in] rt_uint8_t chn - SPI channel number
 *
 * @param[in] const char *name - Pointer to SPI name
 *
 * @param[in] rt_uint8_t cfg - SPI config
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_spi_contex_init(struct bsp_spi_contex *ctx, rt_uint8_t chn,
    const char *name, void *ldev) {
    rt_err_t ret;

    do {
        ctx->chn = chn;
        ctx->start = RT_FALSE;
        SPI_SET_SPEED(ctx, SPI_DEFAULT_SPEED);
        ctx->ldev = ldev;

        /* init lock */
        ret = rt_mutex_init(&ctx->lok, name, RT_IPC_FLAG_FIFO);
        if (RT_EOK != ret) break;

        /* register device */
        ctx->dev.type = RT_Device_Class_SPIBUS;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = RT_NULL;
        ctx->dev.open = bsp_spi_open;
        ctx->dev.close = bsp_spi_close;
        ctx->dev.read = bsp_spi_read;
        ctx->dev.write = bsp_spi_write;
        ctx->dev.control = bsp_spi_control;
        ctx->dev.user_data = (void *)ctx;
        ret = rt_device_register(&ctx->dev, name, RT_DEVICE_FLAG_RDWR);
    } while (0);

    return ret;
}

/* Public functions ----------------------------------------------------------*/
/***************************************************************************//**
 * @brief - initialize SPI contex and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_spi_init(void) {
    rt_uint8_t spi_chs[SPI_CH_NUM] = {
        #if CONFIG_USING_SPI0
        SPI_CH0,
        #endif
        #if CONFIG_USING_SPI1
        SPI_CH1,
        #endif
    };
    void *ldev;
    const char *name;
    rt_uint8_t i;
    rt_uint8_t chn;
    rt_err_t ret;

    for (i = 0; i < sizeof(spi_chs); i++) {
        chn = spi_chs[i];

        switch (chn) {
        #if CONFIG_USING_SPI0
        case 0:
            ldev = (void *)&_SPI0;
            name = SPI_NAME(0);
            break;
        #endif
        #if CONFIG_USING_SPI1
        case 1:
            ldev = (void *)&_SPI1;
            name = SPI_NAME(1);
            break;
        #endif
        default:
            return -RT_ERROR;
        } 

        ret = bsp_spi_contex_init(&SPI_CTX(i), chn, name, ldev);
        if (RT_EOK != ret) break;

        SPI_DEV(&SPI_CTX(i))->begin();
        LOG_D("[SPI%d] h/w init ok!", chn);
    }

    if (RT_EOK != ret) {
        LOG_E("[SPI%d E] h/w init failed!", chn);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && !CONFIG_USING_DRIVER_SPI && (CONFIG_USING_SPI0 || CONFIG_USING_SPI1) */

} /* extern "C" */
