/***************************************************************************//**
 * @file    drv_iic.cpp
 * @brief   Arduino RT-Thread library IIC device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && (CONFIG_USING_IIC0 || CONFIG_USING_IIC1)
}

#include <Arduino.h>
#include <Wire.h>   /* Arduino library */

extern "C" {

#include "drv_iic.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_IIC_DEBUG
#  define LOG_LVL                   LOG_LVL_DBG
# else
#  define LOG_LVL                   LOG_LVL_INFO
# endif
# define LOG_TAG                    "IIC"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# ifdef BSP_IIC_DEBUG
#  define LOG_I(format, args...)    rt_kprintf(format "\n", ##args)
# else
#  define LOG_I(format, args...)
# endif
# define LOG_D                      LOG_I
# define LOG_HEX(format, args...)
#endif /* RT_USING_ULOG */

#if CONFIG_USING_IIC0
# define _IIC0                      Wire
#endif
#if CONFIG_USING_IIC1
# define _IIC1                      Wire1
#endif
#define IIC_NAME(ch)                "IIC"#ch
#define IIC_CTX(idx)                iic_ctx[idx]
#define IIC_DEV(ctx)                ((TwoWire *)((ctx)->ldev))
#define IIC_TX_START(ctx, addr)     \
    LOG_D("[IIC%d] TX START: %02x", ctx->chn, addr); \
    IIC_DEV(ctx)->beginTransmission(addr)
#define IIC_TX_STOP(ctx)            \
        LOG_D("[IIC%d] TX STOP", ctx->chn); \
        (void)IIC_DEV(ctx)->endTransmission()
#define IIC_TX(ctx, data)           (void)IIC_DEV(ctx)->write((rt_uint8_t)data)
#define IIC_RX_START(ctx, addr, sz, stop) \
    LOG_D("[IIC%d] RX START: %02x, %d, %d", ctx->chn, addr, sz, stop); \
    IIC_DEV(ctx)->requestFrom(addr, sz, stop)
#define IIC_RX(ctx, buf) {              \
    while (!IIC_DEV(ctx)->available()); \
    *(buf) = IIC_DEV(ctx)->read(); \
}
#define SLAVE_ADDR(addr)            (rt_uint8_t)(addr & 0x000000ff)
#define ACCESS_ADDR(flags)          (rt_uint8_t)((flags & 0x0000ff00) >> 8)
#define MAX_RX_SIZE                 (0xff)

/* Private variables ---------------------------------------------------------*/
static struct bsp_iic_contex iic_ctx[IIC_CH_NUM];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static rt_err_t bsp_iic_open(rt_device_t dev, rt_uint16_t oflag) {
    struct bsp_iic_contex *ctx = (struct bsp_iic_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != ret) break;

        ctx->dev.open_flag = oflag & RT_DEVICE_OFLAG_MASK;
        LOG_D("[IIC%d] open with flag %x", ctx->chn, oflag);
    } while (0);

    if (RT_EOK != ret) {
        LOG_W("[IIC%d E] open err [%08x]", ctx->chn, ret);
    }

    return ret;
}

static rt_err_t bsp_iic_close(rt_device_t dev) {
    struct bsp_iic_contex *ctx = (struct bsp_iic_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_release(&ctx->lok);
        if (RT_EOK != ret) break;

        LOG_D("[IIC%d] closed", ctx->chn);
    } while (0);

    if (RT_EOK != ret) {
        LOG_W("[IIC%d E] close err [%08x]", ctx->chn, ret);
    }
    return ret;
}

static rt_size_t bsp_iic_read(rt_device_t dev, rt_off_t _addr, void *_buf,
    rt_size_t size) {
    struct bsp_iic_contex *ctx = (struct bsp_iic_contex *)(dev->user_data);
    rt_bool_t locked;
    rt_err_t err;
    rt_size_t ret;

    if (RT_NULL == _addr) return -RT_EINVAL;

    do {
        rt_uint8_t i;
        rt_uint8_t slave = SLAVE_ADDR(_addr);
        rt_uint8_t addr = ACCESS_ADDR(_addr);
        rt_uint8_t *buf = (rt_uint8_t *)_buf;

        locked = RT_FALSE;
        ret = 0;

        if ((RT_NULL == _addr) || \
            (RT_DEVICE_OFLAG_WRONLY == (ctx->dev.open_flag & 0x0003))) {
            err = -RT_EINVAL;
            break;
        }

        err = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != err) break;
        locked = RT_TRUE;

        /* send slave addr */
        IIC_TX_START(ctx, slave);
        IIC_TX(ctx, addr);
        IIC_TX_STOP(ctx);

        /* receive data */
        ret = size;
        while (size) {
            rt_uint8_t stop = (size > MAX_RX_SIZE) ? 0 : 1;
            rt_uint8_t sz = stop ? size : MAX_RX_SIZE;
            IIC_RX_START(ctx, slave, sz, stop);
            for (i = 0; i < sz; i++) {
                IIC_RX(ctx, buf++);
            }
            size -= sz;
        }
    } while (0);

    if (locked) rt_mutex_release(&ctx->lok);
    if (RT_EOK != err) rt_set_errno(err);

    return ret;
}

static rt_size_t bsp_iic_write(rt_device_t dev, rt_off_t _addr,
    const void *_buf, rt_size_t size) {
    struct bsp_iic_contex *ctx = (struct bsp_iic_contex *)(dev->user_data);
    rt_bool_t locked;
    rt_err_t err;
    rt_size_t ret;

    if (RT_NULL == _addr) return -RT_EINVAL;
    if (RT_DEVICE_OFLAG_RDONLY == (ctx->dev.open_flag & 0x0003))
        return -RT_EINVAL;

    do {
        rt_size_t i;
        rt_uint8_t slave = SLAVE_ADDR(_addr);
        rt_uint8_t addr = ACCESS_ADDR(_addr);
        rt_uint8_t *buf = (rt_uint8_t *)_buf;

        locked = RT_FALSE;
        ret = 0;

        err = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != err) break;
        locked = RT_TRUE;

        /* send slave addr and data */
        IIC_TX_START(ctx, slave);
        IIC_TX(ctx, addr);
        for (i = 0; i < size; i++) {
            IIC_TX(ctx, *(buf++));
        }
        IIC_TX_STOP(ctx);

        ret = size;
    } while (0);

    if (locked) rt_mutex_release(&ctx->lok);
    if (RT_EOK != err) rt_set_errno(err);

    return ret;
}

/***************************************************************************//**
 * @brief Initialize IIC contex
 *
 * @param[in] struct bsp_iic_contex *ctx - Pointer to IIC contex
 *
 * @param[in] rt_uint8_t chn - IIC channel number
 *
 * @param[in] const char *name - Pointer to IIC name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_iic_contex_init(struct bsp_iic_contex *ctx, rt_uint8_t chn,
    const char *name, void *ldev) {
    rt_err_t ret;

    do {
        ctx->chn = chn;
        ctx->ldev = ldev;

        /* init lock */
        ret = rt_mutex_init(&ctx->lok, name, RT_IPC_FLAG_FIFO);
        if (RT_EOK != ret) break;

        /* register device */
        ctx->dev.type = RT_Device_Class_I2CBUS;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = RT_NULL;
        ctx->dev.open = bsp_iic_open;
        ctx->dev.close = bsp_iic_close;
        ctx->dev.read = bsp_iic_read;
        ctx->dev.write = bsp_iic_write;
        ctx->dev.control = RT_NULL;
        ctx->dev.user_data = (void *)ctx;
        ret = rt_device_register(&ctx->dev, name, RT_DEVICE_FLAG_RDWR);
    } while (0);

    return ret;
}

/* Public functions ----------------------------------------------------------*/
/***************************************************************************//**
 * @brief - initialize IIC contex and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_iic_init(void) {
    rt_uint8_t iic_chs[IIC_CH_NUM] = {
        #if CONFIG_USING_IIC0
        IIC_CH0,
        #endif
        #if CONFIG_USING_IIC1
        IIC_CH1,
        #endif
    };
    void *ldev;
    const char *name;
    rt_uint8_t i;
    rt_uint8_t chn;
    rt_err_t ret;

    for (i = 0; i < sizeof(iic_chs); i++) {
        chn = iic_chs[i];

        switch (chn) {
        #if CONFIG_USING_IIC0
        case 0:
            ldev = (void *)&_IIC0;
            name = IIC_NAME(0);
            break;
        #endif
        #if CONFIG_USING_IIC1
        case 1:
            ldev = (void *)&_IIC1;
            name = IIC_NAME(1);
            break;
        #endif
        default:
            return -RT_ERROR;
        } 

        ret = bsp_iic_contex_init(&IIC_CTX(i), chn, name, ldev);
        if (RT_EOK != ret) break;

        IIC_DEV(&IIC_CTX(i))->begin();
        LOG_D("[IIC%d] h/w init ok!", chn);
    }

    if (RT_EOK != ret) {
        LOG_E("[IIC%d E] h/w init failed!", chn);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && (CONFIG_USING_IIC0 || CONFIG_USING_IIC1) */

} /* extern "C" */
