/***************************************************************************//**
 * @file    drv_spi.cpp
 * @brief   Arduino RT-Thread library SPI device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && (CONFIG_USING_SPI0 || CONFIG_USING_SPI1)
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
#ifdef BSP_SPI_DEBUG
#define spi_debug(format, args...)  rt_kprintf(format, ##args)
#else
#define spi_debug(format, args...)
#endif

#if CONFIG_USING_SPI0
#define SPI0                        SPI
#endif
#define SPI_NAME(ch)                "SPI"#ch
#define SPI_CTX(idx)                spi_ctx[idx]
#define SPI_DEV(ctx)                ((SPIClass *)((ctx)->ldev))
#define SPI_START(ctx)              spi_debug("SPI%d: START\n", ctx->chn); \
                                    SPI_DEV(ctx)->beginTransaction(ctx->set)
#define SPI_STOP(ctx)               spi_debug("SPI%d: STOP\n", ctx->chn); \
                                    SPI_DEV(ctx)->endTransaction()
#define SPI_TX(ctx, data)           (void)SPI_DEV(ctx)->transfer((rt_uint8_t)data)
#define SPI_RX(ctx)                 SPI_DEV(ctx)->transfer(0xff)
#define SPI_SET_SPEED(ctx, target)  \
if (target != ctx->spd) { \
    ctx->set = SPISettings(target, MSBFIRST, SPI_MODE0); \
    ctx->spd = target; \
    spi_debug("SPI%d: speed=%d\n", ctx->chn, target); \
}

/* Private variables ---------------------------------------------------------*/
static struct bsp_spi_contex spi_ctx[CH_NUM];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static rt_bool_t wait_idle(struct bsp_spi_contex *ctx) {
    rt_uint32_t i;

    for (i = 0; i < SPI_DEFAULT_LIMIT; i++) {
        if (0xff == SPI_RX(ctx)) break;
    }

    spi_debug("SPI%d: wait %d\n", ctx->chn, i);
    return (i < SPI_DEFAULT_LIMIT);
}

static rt_bool_t wait_token(struct bsp_spi_contex *ctx, rt_uint8_t token) {
    rt_uint32_t i;

    for (i = 0; i < SPI_DEFAULT_LIMIT; i++) {
        if (token == SPI_RX(ctx)) break;
    }

    spi_debug("SPI%d: wait token %d\n", ctx->chn, i);
    return (i < SPI_DEFAULT_LIMIT);
}

static rt_err_t bsp_spi_open(rt_device_t dev, rt_uint16_t oflag) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != ret) break;

        ctx->dev.open_flag = oflag & RT_DEVICE_OFLAG_MASK;
        spi_debug("SPI%d: open with flag %x\n", ctx->chn, oflag);
    } while (0);

    if (RT_EOK != ret)
        spi_debug("SPI%d err: open failed [%08x]\n", ctx->chn, ret);
    return ret;
}

static rt_err_t bsp_spi_close(rt_device_t dev) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_release(&ctx->lok);
        if (RT_EOK != ret) break;

        spi_debug("SPI%d: closed\n", ctx->chn);
    } while (0);

    if (RT_EOK != ret)
        spi_debug("SPI%d err: close failed [%08x]\n", ctx->chn, ret);
    return ret;
}

static rt_size_t bsp_spi_read(rt_device_t dev, rt_off_t token, void *buf,
    rt_size_t size) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_bool_t locked;
    rt_err_t err;
    rt_size_t ret;

    if (RT_DEVICE_OFLAG_WRONLY == (ctx->dev.open_flag & 0x0003))
        return -RT_EINVAL;

    do {
        rt_size_t i;

        locked = RT_FALSE;
        ret = 0;

        err = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != err) break;
        locked = RT_TRUE;

        if (RT_NULL == buf) {
            /* dummy read */
            spi_debug("SPI%d: dummy read [%d]\n", ctx->chn, size);
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
            for (i = 0; i < SPI_DEFAULT_RETRY; i++)
                if (wait_idle(ctx)) break;
            if (i >= SPI_DEFAULT_RETRY) {
                SPI_STOP(ctx);
                err = -RT_EBUSY;
                spi_debug("SPI%d err: read busy\n", ctx->chn);
                break;
            }

            /* send instruction */
            spi_debug("SPI%d: tx ins [%d]\n", ctx->chn, inst_len);
            for (i = 0; i < inst_len; i++)
                SPI_TX(ctx, *(inst_ptr + i));

            /* receive data */
            spi_debug("SPI%d: rx data [%d]\n", ctx->chn, size);
            if (0 != token) {
                for (i = 0; i < SPI_DEFAULT_RETRY; i++)
                    if (wait_token(ctx, (rt_uint8_t)token)) break;
                if (i >= SPI_DEFAULT_RETRY) {
                    SPI_STOP(ctx);
                    err = -RT_EIO;
                    spi_debug("SPI%d err: no token\n", ctx->chn);
                    break;
                }
            }
            for (i = 0; i < size; i++)
                *(rx_buf++) = SPI_RX(ctx);
            SPI_STOP(ctx);

            ret = size;
        }
    } while (0);

    if (locked) rt_mutex_release(&ctx->lok);
    if (RT_EOK != err) rt_set_errno(err);

    return ret;
}

static rt_size_t bsp_spi_write(rt_device_t dev, rt_off_t pos, const void *buf,
    rt_size_t size) {
    struct bsp_spi_contex *ctx = (struct bsp_spi_contex *)(dev->user_data);
    rt_bool_t locked;
    rt_err_t err;
    rt_size_t ret;

    (void)pos;
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
        for (i = 0; i < SPI_DEFAULT_RETRY; i++)
            if (wait_idle(ctx)) break;
        if (i >= SPI_DEFAULT_RETRY) {
            SPI_STOP(ctx);
            err = -RT_EBUSY;
            spi_debug("SPI%d err: write busy\n", ctx->chn);
            break;
        }

        /* send instruction */
        spi_debug("SPI%d: tx ins [%d]\n", ctx->chn, inst_len);
        for (i = 0; i < inst_len; i++)
            SPI_TX(ctx, *(inst_ptr + i));

        /* send data */
        spi_debug("SPI%d: tx data [%d]\n", ctx->chn, size);
        for (i = 0; i < size; i++)
            SPI_TX(ctx, *(tx_buf + i));
        SPI_STOP(ctx);

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
static rt_err_t bsp_spi_contex_init(struct bsp_spi_contex *ctx,
    rt_uint8_t chn, const char *name, rt_uint8_t cfg, void *ldev) {
    rt_err_t ret;

    do {
        ctx->chn = chn;
        ctx->cfg = cfg;
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
    rt_uint8_t spi_chs[CH_NUM] = {
        #if CONFIG_USING_SPI0
        CH0,
        #endif
        #if CONFIG_USING_SPI1
        CH1,
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
            ldev = (void *)&SPI0;
            name = SPI_NAME(0);
            break;
        #endif
        #if CONFIG_USING_SPI1
        case 1:
            ldev = (void *)&SPI1;
            name = SPI_NAME(1);
            break;
        #endif
        default:
            return -RT_ERROR;
        } 

        ret = bsp_spi_contex_init(&SPI_CTX(i), chn, name, SPI_DEFAULT_CONFIG,
            ldev);
        if (RT_EOK != ret) break;

        SPI_DEV(&SPI_CTX(i))->begin();
        spi_debug("SPI%d: h/w init ok!\n", chn);
    }

    if (RT_EOK != ret)
        rt_kprintf("SPI%d err: h/w init failed!\n", chn);

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && (CONFIG_USING_SPI0 || CONFIG_USING_SPI1) */

} /* extern "C" */
