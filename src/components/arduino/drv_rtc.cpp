/***************************************************************************//**
 * @file    drv_rtc.cpp
 * @brief   Arduino RT-Thread library RTC device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if (defined(CONFIG_ARDUINO) && CONFIG_USING_DRIVER_RTC)
}

#include <time.h>
#include <Arduino.h>
#ifdef ARDUINO_ARCH_SAMD
# include <RTCZero.h>   /* Arduino library */
#endif

extern "C" {

#include "components/drivers/include/rtdevice.h"
#include "drv_rtc.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_RTC_DEBUG
#  define LOG_LVL                   LOG_LVL_DBG
# else
#  define LOG_LVL                   LOG_LVL_INFO
# endif
# define LOG_TAG                    "RTC"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# ifdef BSP_RTC_DEBUG
#  define LOG_I(format, args...)    rt_kprintf(format "\n", ##args)
# else
#  define LOG_I(format, args...)
# endif
# define LOG_D                      LOG_I
# define LOG_HEX(format, args...)
#endif /* RT_USING_ULOG */

#define RTC_CTX()                   (&rtc_ctx)
#ifdef ARDUINO_ARCH_SAMD
# define RTC_DEV(ctx)               ((RTCZero *)((ctx)->ldev))
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_rtc_contex rtc_ctx;
#ifdef ARDUINO_ARCH_SAMD
}
static RTCZero _rtc;
extern "C" {
#endif /* ARDUINO_ARCH_SAMD */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static rt_err_t bsp_rtc_open(rt_device_t dev, rt_uint16_t oflag) {
    struct bsp_rtc_contex *ctx = (struct bsp_rtc_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
        if (RT_EOK != ret) break;

        ctx->dev.open_flag = oflag & RT_DEVICE_OFLAG_MASK;
        LOG_D("[RTC] open with flag %x", oflag);
    } while (0);

    if (RT_EOK != ret) {
        LOG_W("[RTC E] open err [%08x]", ret);
    }

    return ret;
}

static rt_err_t bsp_rtc_close(rt_device_t dev) {
    struct bsp_rtc_contex *ctx = (struct bsp_rtc_contex *)(dev->user_data);
    rt_err_t ret;

    do {
        ret = rt_mutex_release(&ctx->lok);
        if (RT_EOK != ret) break;

        LOG_D("[RTC] closed");
    } while (0);

    if (RT_EOK != ret) {
        LOG_W("[RTC E] close err [%08x]", ret);
    }
    return ret;
}

static rt_err_t bsp_rtc_control(rt_device_t dev, rt_int32_t cmd, void *args) {
    struct bsp_rtc_contex *ctx = (struct bsp_rtc_contex *)(dev->user_data);
    time_t *timep;
    struct rt_rtc_wkalarm *alarm;
    rt_err_t ret;

    /* lock */
    ret = rt_mutex_take(&ctx->lok, RT_WAITING_NO);
    if (RT_EOK != ret) return ret;

    do {
        /* process cmd */
        switch (cmd) {
        case RT_DEVICE_CTRL_RTC_GET_TIME:
            timep = (time_t *)args;
            #ifdef ARDUINO_ARCH_SAMD
                *timep = (time_t)RTC_DEV(RTC_CTX())->getEpoch();
            #endif
            ret = RT_EOK;
            break;

        case RT_DEVICE_CTRL_RTC_SET_TIME:
            timep = (time_t *)args;
            #ifdef ARDUINO_ARCH_SAMD
                RTC_DEV(RTC_CTX())->setEpoch((rt_uint32_t)*timep);
            #endif
            ret = RT_EOK;
            break;

        case RT_DEVICE_CTRL_RTC_GET_ALARM:
            alarm = (struct rt_rtc_wkalarm *)args;
            #ifdef ARDUINO_ARCH_SAMD
                alarm->enable = RT_FALSE;
                alarm->tm_hour = RTC_DEV(RTC_CTX())->RTCZero::getAlarmHours();
                alarm->tm_min = RTC_DEV(RTC_CTX())->RTCZero::getAlarmMinutes();
                alarm->tm_sec = RTC_DEV(RTC_CTX())->RTCZero::getAlarmSeconds();
            #endif
            ret = RT_EOK;
            break;

        case RT_DEVICE_CTRL_RTC_SET_ALARM:
            alarm = (struct rt_rtc_wkalarm *)args;
            #ifdef ARDUINO_ARCH_SAMD
                if (!alarm->enable) {
                    RTC_DEV(RTC_CTX())->disableAlarm();
                } else {
                    RTC_DEV(RTC_CTX())->setAlarmTime((rt_uint8_t)alarm->tm_hour,
                        (rt_uint8_t)alarm->tm_min, (rt_uint8_t)alarm->tm_sec);
                    RTC_DEV(RTC_CTX())->enableAlarm(
                        RTC_DEV(RTC_CTX())->MATCH_HHMMSS);
                }
            #endif
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
 * @brief Initialize RTC contex
 *
 * @param[in] struct bsp_rtc_contex *ctx - Pointer to RTC contex
 *
 * @param[in] const char *name - Pointer to RTC name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_rtc_contex_init(struct bsp_rtc_contex *ctx,
    const char *name, void *ldev) {
    rt_err_t ret;

    do {
        ctx->ldev = ldev;

        /* init lock */
        ret = rt_mutex_init(&ctx->lok, name, RT_IPC_FLAG_FIFO);
        if (RT_EOK != ret) break;

        /* register device */
        ctx->dev.type = RT_Device_Class_RTC;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = RT_NULL;
        ctx->dev.open = bsp_rtc_open;
        ctx->dev.close = bsp_rtc_close;
        ctx->dev.read = RT_NULL;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_rtc_control;
        ctx->dev.user_data = (void *)ctx;
        ret = rt_device_register(&ctx->dev, name, RT_DEVICE_FLAG_RDWR);
    } while (0);

    return ret;
}

/* Public functions ----------------------------------------------------------*/
/***************************************************************************//**
 * @brief - initialize RTC contex and hardware
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_rtc_init(void) {
    rt_err_t ret;

    do {
        #ifdef ARDUINO_ARCH_SAMD
            ret = bsp_rtc_contex_init(RTC_CTX(), RTC_NAME, &_rtc);
            if (RT_EOK != ret) break;

            RTC_DEV(RTC_CTX())->begin();
        #endif
        LOG_D("[RTC] h/w init ok!");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[RTC E] h/w init failed!");
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* (defined(CONFIG_ARDUINO) && CONFIG_USING_DRIVER_RTC) */

} /* extern "C" */
