/***************************************************************************//**
 * @file    drv_tick.cpp
 * @brief   Arduino RT-Thread library RISC-V tick driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include "components/drivers/include/rtdevice.h"

#if defined(BOARD_SIPEED_LONGAN_NANO) && CONFIG_USING_DRIVER_SPI

#include "bsp/bsp.h"

#if !CONFIG_USING_SPI0 && !CONFIG_USING_SPI1
# error "No CONFIG_USING_SPIx enabled"
#endif

/***************************************************************************//**
 * @addtogroup LonganNano
 * @{
 ******************************************************************************/

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

/* Private typedef -----------------------------------------------------------*/
struct bsp_spi_contex {
    char *name;
    rcu_clock_freq_enum clk_src;
    rt_uint32_t chn;
    rt_uint32_t spi_base;
    rt_uint32_t spi_clk;
    rt_uint32_t gpio_clk;
    rt_uint32_t gpio_port;
    rt_uint32_t mosi_pin;
    rt_uint32_t miso_pin;
    rt_uint32_t sck_pin;
    rt_uint32_t cs_pin;
    // IRQn_Type irq;
};

enum bsp_spi_channel {
    #if CONFIG_USING_SPI0
    SPI_CH0 = 0,
    #endif
    #if CONFIG_USING_SPI1
    SPI_CH1 = 1,
    #endif
    SPI_CH_NUM = CONFIG_USING_SPI0 + CONFIG_USING_SPI1,
};

/* Private function prototypes -----------------------------------------------*/
static rt_err_t op_configure(struct rt_spi_device* dev,
    struct rt_spi_configuration* cfg);
static rt_uint32_t op_xfer(struct rt_spi_device* dev,
    struct rt_spi_message* msg);

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static struct rt_spi_bus spi_dev[SPI_CH_NUM];

static struct bsp_spi_contex spi_ctx[SPI_CH_NUM] = {
    #if CONFIG_USING_SPI0
    {
        "SPI0",
        CK_APB2,
        SPI_CH0,
        SPI0,
        RCU_SPI0,
        RCU_GPIOA,
        GPIOA,
        GPIO_PIN_7,
        GPIO_PIN_6,
        GPIO_PIN_5,
        GPIO_PIN_4,
    },
    #endif
    #if CONFIG_USING_SPI1
    {
        "SPI1",
        CK_APB1,
        SPI_CH1,
        SPI1,
        RCU_SPI1,
        RCU_GPIOB,
        GPIOB,
        GPIO_PIN_15,
        GPIO_PIN_14,
        GPIO_PIN_13,
        GPIO_PIN_12,
    },
    #endif
};

static struct rt_spi_ops spi_ops = {
    op_configure,
    op_xfer,
};

/* Private functions ---------------------------------------------------------*/
static rt_err_t op_configure(struct rt_spi_device* dev,
    struct rt_spi_configuration* cfg) {
    struct bsp_spi_contex *ctx;
    spi_parameter_struct init;
    rt_uint32_t clk;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    ctx = (struct bsp_spi_contex *)dev->bus->parent.user_data
    RT_ASSERT(ctx != RT_NULL);
    LOG_D("[SPI%d] op_configure", ctx->chn);

    spi_i2s_deinit(ctx->spi_base);

    /* NOTE: currently only support master mode */
    init.device_mode = SPI_MASTER;

    if (cfg->mode & RT_SPI_3WIRE) {
        init.trans_mode = SPI_TRANSMODE_BDTRANSMIT;
    } else {
        init.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    }

    if (cfg->data_width <= 8) {
        init.frame_size = SPI_FRAMESIZE_8BIT;
    } else if(cfg->data_width <= 16) {
        init.frame_size = SPI_FRAMESIZE_16BIT;
    } else {
        LOG_W("[SPI%d E] data_width (%d)", ctx->chn, cfg->data_width);
        return RT_EINVAL;
    }

    /* NOTE: currently only support software CS */
    init.nss = SPI_NSS_SOFT;

    if (cfg->mode & RT_SPI_MSB) {
        init.endian = SPI_ENDIAN_MSB;
    } else {
        init.endian = SPI_ENDIAN_LSB;
    }

    switch (cfg->mode & (RT_SPI_CPOL | RT_SPI_CPHA)) {
    case RT_SPI_MODE_0:
        init.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        break;
    case RT_SPI_MODE_1:
        init.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
        break;
    case RT_SPI_MODE_2:
        init.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
        break;
    case RT_SPI_MODE_3:
        init.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
        break;
    default:
        break;
    }

    if (0 == ctx->chn) {
        clk = rcu_clock_freq_get(CK_APB2);
    } else {
        clk = rcu_clock_freq_get(CK_APB1);
    }
    LOG_D("[SPI%d] APB freq: %7d\n", ctx->chn, clk);
    LOG_D("[SPI%d] MAX freq: %7d\n", ctx->chn, cfg->max_hz);
    if (cfg->max_hz >= clk >> 1) {
        init.prescale = SPI_PSC_2;
    } else if (cfg->max_hz >= clk >> 2) {
        init.prescale = SPI_PSC_4;
    } else if (cfg->max_hz >= clk >> 3) {
        init.prescale = SPI_PSC_8;
    } else if (cfg->max_hz >= clk >> 4) {
        init.prescale = SPI_PSC_16;
    } else if (cfg->max_hz >= clk >> 5) {
        init.prescale = SPI_PSC_32;
    } else if (cfg->max_hz >= clk >> 6) {
        init.prescale = SPI_PSC_64;
    } else if (cfg->max_hz >= clk >> 7) {
        init.prescale = SPI_PSC_128;
    } else {
        init.prescale = SPI_PSC_256;
    }

    spi_init(ctx->spi_base, &init);
    spi_crc_off(ctx->spi_base);
    spi_enable(ctx->spi_base);

    return RT_EOK;
};

static rt_uint32_t op_xfer(struct rt_spi_device* dev,
    struct rt_spi_message* msg) {
    struct bsp_spi_contex *ctx;
    struct rt_spi_configuration *cfg;
    rt_uint8_t mode3 = 0;
    rt_uint32_t i;
    const rt_uint16_t dummy = 0xffff;

    RT_ASSERT(dev != NULL);
    RT_ASSERT(msg != NULL);
    ctx = (struct bsp_spi_contex *)dev->bus->parent.user_data;
    cfg = &dev->config;

    if (RT_NULL != msg->send_buf) {
        if (cfg->mode & RT_SPI_3WIRE) {
            spi_bidirectional_transfer_config(ctx->spi_base,
                SPI_BIDIRECTIONAL_TRANSMIT);
            mode3 = 1;
            LOG_D("[SPI%d] 3TX (%d)", ctx->chn, msg->length);
        } else {
            LOG_D("[SPI%d] TX (%d)", ctx->chn, msg->length);
        }
    }
    if (RT_NULL != msg->recv_buf) {
        if ((mode3 == 0) && (cfg->mode & RT_SPI_3WIRE)) {
            spi_bidirectional_transfer_config(ctx->spi_base,
                SPI_BIDIRECTIONAL_RECEIVE);
            mode3 = 2;
            LOG_D("[SPI%d] 3RX (%d)", ctx->chn, msg->length);
        } else {
            LOG_D("[SPI%d] RX (%d)", ctx->chn, msg->length);
        }
    }

    if (msg->cs_take) {
        LOG_D("[SPI%d] CS enable", ctx->chn);
        gpio_bit_reset(ctx->gpio_port, ctx->cs_pin);
    }

    for (i = 0; i < msg->length; i++) {
        if (2 != mode3) {
            while (RESET == spi_i2s_flag_get(ctx->spi_base, SPI_FLAG_TBE)) {
                asm volatile ("nop");
            }
            if (RT_NULL == msg->send_buf) {
                spi_i2s_data_transmit(ctx->spi_base, dummy);
            } else if (cfg->data_width <= 8) {
                spi_i2s_data_transmit(ctx->spi_base,
                    (rt_uint16_t)((rt_uint8_t *)msg->send_buf)[i]);
            } else {
                spi_i2s_data_transmit(ctx->spi_base,
                    ((rt_uint16_t *)msg->send_buf)[i]);
            }
        }

        if (1 != mode3) {
            while (RESET == spi_i2s_flag_get(ctx->spi_base, SPI_FLAG_RBNE)) {
                asm volatile ("nop");
            }
            if (RT_NULL == msg->recv_buf) {
                (void)spi_i2s_data_receive(ctx->spi_base);
            } else if (cfg->data_width <= 8) {
                ((rt_uint8_t *)msg->recv_buf)[i] = (rt_uint8_t)spi_i2s_data_receive(ctx->spi_base);
            } else {
                ((rt_uint16_t *)msg->recv_buf)[i] = spi_i2s_data_receive(ctx->spi_base);
            }
        }
    }

    if (msg->cs_release) {
        LOG_D("[SPI%d] CS disable", ctx->chn);
        gpio_bit_set(ctx->gpio_port, ctx->cs_pin);
    }

    return msg->length;
};

/* Public functions ----------------------------------------------------------*/
rt_err_t bsp_hw_spi_init(void) {
    uint8_t i;
    rt_err_t ret;

    ret = RT_ERROR;
    for (i = 0; i < SPI_CH_NUM; i++) {
        /* enable clock */
        rcu_periph_clock_enable(spi_ctx[i].gpio_clk);
        rcu_periph_clock_enable(spi_ctx[i].spi_clk);
        /* setup GPIO */
        gpio_init(spi_ctx[i].gpio_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            spi_ctx[i].mosi_pin | spi_ctx[i].miso_pin | spi_ctx[i].sck_pin);
        gpio_init(spi_ctx[i].gpio_port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
            spi_ctx[i].cs_pin);
        /* register device */
        spi_dev[i].parent.user_data = (void *)&spi_ctx[i];
        ret = rt_spi_bus_register(
            &spi_dev[i],
            spi_ctx[i].name,
            &spi_ops);
        RT_ASSERT(ret == RT_EOK);
        LOG_D("[SPI%d] h/w init ok", spi_ctx[i].chn);
    }

    return ret;
}
INIT_BOARD_EXPORT(bsp_hw_spi_init);

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(BOARD_SIPEED_LONGAN_NANO) && CONFIG_USING_DRIVER_SPI */
