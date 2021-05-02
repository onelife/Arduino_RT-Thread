/***************************************************************************//**
 * @file    drv_button.cpp
 * @brief   Arduino RT-Thread library button device driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_BUTTON
}

#include <Arduino.h>

#if CONFIG_USING_GUI
# include <rttgui.h>
#endif

extern "C" {

#include "drv_button.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
typedef void (*rx_indicator)(void);

/* Private define ------------------------------------------------------------*/
#ifdef RT_USING_ULOG
# ifdef BSP_BUTTON_DEBUG
#  define LOG_LVL                       LOG_LVL_DBG
# else
#  define LOG_LVL                       LOG_LVL_INFO
# endif
# define LOG_TAG                        "BTN"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)         rt_kprintf(format "\n", ##args)
# define LOG_W                          LOG_E
# ifdef BSP_BUTTON_DEBUG
#  define LOG_D(format, args...)        rt_kprintf(format "\n", ##args)
# else
#  define LOG_D(format, args...)
# endif
# define LOG_I                          LOG_D
# define LOG_HEX(format, args...)                    
#endif /* RT_USING_ULOG */

#define BTN_IO_INIT(pin)                pinMode(pin, INPUT_PULLUP)
#define BTN_STATE(pin)                  digitalRead(pin)
#define BTN_CTX()                       (&btn_ctx)

/* Private function prototypes -----------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct bsp_btn_contex btn_ctx;
static rx_indicator key_indicator = RT_NULL;
static rt_uint32_t btn_pin[CONFIG_USING_BUTTON] = CONFIG_BUTTON_PIN;
static rt_uint16_t btn_key[CONFIG_USING_BUTTON] = CONFIG_BUTTON_CODE;
static rt_bool_t btn_state[CONFIG_USING_BUTTON];
static rt_bool_t btn_change[CONFIG_USING_BUTTON];

/* Private functions ---------------------------------------------------------*/
static void _btn_timeout(void *param) {
    struct bsp_btn_contex *ctx = BTN_CTX();
    rt_bool_t changed = RT_FALSE;
    rt_uint32_t i;
    (void)param;

    for (i = 0; i < ctx->num; i++) {
        if (btn_state[i] != BTN_STATE(btn_pin[i])) {
            btn_state[i] = !btn_state[i];
            btn_change[i] = RT_TRUE;
            changed = RT_TRUE;
        }
    }
    if (key_indicator && changed) key_indicator();
}

static rt_size_t bsp_button_read(rt_device_t dev, rt_off_t pos, void *buf,
    rt_size_t size) {
    struct bsp_btn_contex *ctx = (struct bsp_btn_contex *)(dev->user_data);
    rt_uint32_t i;
    (void)dev;
    (void)pos;
    (void)size;

    for (i = 0; i < ctx->num; i++) {
        if (btn_change[i]) {
            #if CONFIG_USING_GUI
                rtgui_key_t *data = (rtgui_key_t *)buf;
                data->key = (rtgui_kbd_key_t)btn_key[i];
                data->mod = RTGUI_KMOD_NONE;
                data->type = btn_state[i] ? RTGUI_KEYUP : RTGUI_KEYDOWN;

            #else
                rt_uint16_t *data = (rt_uint16_t *)buf;
                *data = btn_key[i];

            #endif
            btn_change[i] = RT_FALSE;
            // LOG_D("[BTN] read %d %d", btn_key[i], btn_state[i]);
            return 1;
        }
    }
    return 0;
}

static rt_err_t bsp_button_control(rt_device_t dev, rt_int32_t cmd, void *args) {
    rt_err_t ret = -RT_ERROR;
    (void)dev;

    switch (cmd) {
    case RT_DEVICE_CTRL_SET_RX_INDICATOR:
        key_indicator = (rx_indicator)args;
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
 * @param[in] struct bsp_btn_contex *ctx - Pointer to button contex
 *
 * @param[in] const char *name - Pointer to button device name
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_button_contex_init(struct bsp_btn_contex *ctx,
    const char *name) {
    rt_err_t ret;

    do {
        ctx->num = CONFIG_USING_BUTTON;
        rt_timer_init(&ctx->tmr, name, _btn_timeout, RT_NULL,
            BTN_CHECK_TICK, RT_TIMER_FLAG_PERIODIC);

        /* register device */
        ctx->dev.type = RT_Device_Class_Miscellaneous;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = RT_NULL;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = bsp_button_read;
        ctx->dev.write = RT_NULL;
        ctx->dev.control = bsp_button_control;
        ctx->dev.user_data = (void *)ctx;
        ret = rt_device_register(&ctx->dev, name,
            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
        if (RT_EOK != ret) break;
        ret = rt_timer_start(&ctx->tmr);
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
rt_err_t bsp_hw_button_init(void) {
    rt_err_t ret;

    do {
        rt_uint32_t i;

        ret = bsp_button_contex_init(BTN_CTX(), BTN_NAME);
        if (RT_EOK != ret) break;

        for (i = 0; i < CONFIG_USING_BUTTON; i++) {
            BTN_IO_INIT(btn_pin[i]);
            btn_state[i] = BTN_STATE(btn_pin[i]);
            btn_change[i] = RT_FALSE;
        }
        key_indicator = RT_NULL;
        LOG_D("[BTN] h/w init ok");
    } while (0);

    if (RT_EOK != ret) {
        LOG_E("[BTN E] h/w init failed: %d", ret);
    }
    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_BUTTON */

} /* extern "C" */
