/***************************************************************************//**
 * @file    drv_rtc.h
 * @brief   Arduino RT-Thread library RTC device driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_RTC_H__
#define __DRV_RTC_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define RTC_NAME                    "rtc"

/* Exported types ------------------------------------------------------------*/
struct bsp_rtc_contex {
    #if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
        void *ldev;         /* lower level device (Arduino RTC) */
    #endif
    struct rt_mutex lok;    /* lock */
    struct rt_device dev;   /* RT device */
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_rtc_init(void);

#endif /* __DRV_RTC_H__ */
