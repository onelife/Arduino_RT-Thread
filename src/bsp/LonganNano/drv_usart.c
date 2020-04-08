/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-23     tyustli      first version
 */
#include "components/drivers/include/rtdevice.h"

#if defined(BOARD_SIPEED_LONGAN_NANO) && CONFIG_USING_DRIVER_SERIAL

#include "bsp/bsp.h"

#if !CONFIG_USING_UART0 && !CONFIG_USING_UART1
# error "No CONFIG_USING_UARTx enabled"
#endif

/***************************************************************************//**
 * @addtogroup LonganNano
 * @{
 ******************************************************************************/

/* Private define ------------------------------------------------------------*/
#define NUMBER_OF_USART     (CONFIG_USING_UART0 + CONFIG_USING_UART1)

/* Private typedef -----------------------------------------------------------*/
struct usart_config {
    char *name;
    rt_uint32_t usart_base;
    rt_uint32_t usart_clk;
    rt_uint32_t gpio_clk;
    rt_uint32_t gpio_port;
    rt_uint32_t tx_pin;
    rt_uint32_t rx_pin;
    IRQn_Type irq;
    struct rt_serial_device serial;
};

enum {
    #if CONFIG_USING_UART0
    IDX_USART0,
    #endif
};

/* Private function prototypes -----------------------------------------------*/
static rt_err_t op_configure(struct rt_serial_device *serial,
    struct serial_configure *cfg);
static rt_err_t op_control(struct rt_serial_device *serial, int cmd, void *arg);
static int op_putc(struct rt_serial_device *serial, char ch);
static int op_getc(struct rt_serial_device *serial);

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static struct usart_config usart_cfg[NUMBER_OF_USART] = {
    #if CONFIG_USING_UART0
    {
        "Serial",
        USART0,
        RCU_USART0,
        RCU_GPIOA,
        GPIOA,
        GPIO_PIN_9,
        GPIO_PIN_10,
        USART0_IRQn,
    },
    #endif
};

static const struct rt_uart_ops usart_ops = {
    op_configure,
    op_control,
    op_putc,
    op_getc,
    RT_NULL
};

/* Private functions ---------------------------------------------------------*/
static rt_err_t op_configure(struct rt_serial_device *serial,
    struct serial_configure *cfg) {
    struct usart_config *usart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    usart = (struct usart_config *)serial->parent.user_data;
    RT_ASSERT(usart != RT_NULL);

    /* enable GPIO clock */
    rcu_periph_clock_enable(usart->gpio_clk);
    /* enable USART clock */
    rcu_periph_clock_enable(usart->usart_clk);
    /* connect port to USARTx_Tx */
    gpio_init(usart->gpio_port, GPIO_MODE_AF_PP, GPIO_OSPEED_10MHZ,
            usart->tx_pin);
    /* connect port to USARTx_Rx */
    gpio_init(usart->gpio_port, GPIO_MODE_IN_FLOATING, 0x00, usart->rx_pin);

    usart_deinit(usart->usart_base);
    usart_baudrate_set(usart->usart_base, cfg->baud_rate);

    switch (cfg->data_bits) {
    case DATA_BITS_8:
        usart_word_length_set(usart->usart_base, USART_WL_8BIT);
        break;

    case DATA_BITS_9:
        usart_word_length_set(usart->usart_base, USART_WL_9BIT);
        break;

    default:
        usart_word_length_set(usart->usart_base, USART_WL_8BIT);
        break;
    }

    switch (cfg->stop_bits) {
    case STOP_BITS_1:
        usart_stop_bit_set(usart->usart_base, USART_STB_1BIT);
        break;

    case STOP_BITS_2:
        usart_stop_bit_set(usart->usart_base, USART_STB_2BIT);
        break;

    default:
        usart_stop_bit_set(usart->usart_base, USART_STB_1BIT);
        break;
    }

    switch (cfg->parity) {
    case PARITY_NONE:
        usart_parity_config(usart->usart_base, USART_PM_NONE);
        break;

    case PARITY_ODD:
        usart_parity_config(usart->usart_base, USART_PM_ODD);
        break;

    case PARITY_EVEN:
        usart_parity_config(usart->usart_base, USART_PM_EVEN);
        break;

    default:
        usart_parity_config(usart->usart_base, USART_PM_NONE);
        break;
    }

    usart_hardware_flow_rts_config(usart->usart_base, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(usart->usart_base, USART_CTS_DISABLE);
    usart_receive_config(usart->usart_base, USART_RECEIVE_ENABLE);
    usart_transmit_config(usart->usart_base, USART_TRANSMIT_ENABLE);
    usart_enable(usart->usart_base);

    return RT_EOK;
}

static rt_err_t op_control(struct rt_serial_device *serial, int cmd, void *arg) {
    struct usart_config *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct usart_config *)serial->parent.user_data;
    RT_ASSERT(usart != RT_NULL);

    switch (cmd) {
    case RT_DEVICE_CTRL_CLR_INT:
        eclic_irq_disable(usart->usart_base);
        usart_interrupt_disable(usart->usart_base, USART_INT_RBNE);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        eclic_set_nlbits(ECLIC_GROUP_LEVEL3_PRIO1);
        eclic_irq_enable(usart->irq, 1, 0);
        /* enable receive interrupt */
        usart_interrupt_enable(usart->usart_base, USART_INT_RBNE);
        break;
    }

    return RT_EOK;
}

static int op_putc(struct rt_serial_device *serial, char ch) {
    struct usart_config *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct usart_config *) serial->parent.user_data;
    RT_ASSERT(usart != RT_NULL);

    usart_data_transmit(usart->usart_base, (uint8_t)ch);
    while (RESET == usart_flag_get(usart->usart_base, USART_FLAG_TC)) {
        asm volatile ("nop");
    }

    return 1;
}

static int op_getc(struct rt_serial_device *serial) {
    struct usart_config *usart;
    int ch;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct usart_config *) serial->parent.user_data;
    RT_ASSERT(usart != RT_NULL);

    ch = -1;
    if (RESET != usart_flag_get(usart->usart_base, USART_FLAG_RBNE)) {
        ch = usart_data_receive(usart->usart_base) & 0xff;
    }

    return ch;
}

static void usart_isr(struct rt_serial_device *serial) {
    struct usart_config *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct usart_config *) serial->parent.user_data;
    RT_ASSERT(usart != RT_NULL);

    if ((RESET != usart_interrupt_flag_get(usart->usart_base, USART_INT_FLAG_RBNE)) && \
        (RESET != usart_flag_get(usart->usart_base, USART_FLAG_RBNE))) {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        usart_interrupt_flag_clear(usart->usart_base, USART_INT_FLAG_RBNE);
        usart_flag_clear(usart->usart_base, USART_FLAG_RBNE);
    } else {
        if (RESET != usart_flag_get(usart->usart_base, USART_FLAG_CTSF)) {
            usart_flag_clear(usart->usart_base, USART_FLAG_CTSF);
        }
        if (RESET != usart_flag_get(usart->usart_base, USART_FLAG_LBDF)) {
            usart_flag_clear(usart->usart_base, USART_FLAG_LBDF);
        }
        if (RESET != usart_flag_get(usart->usart_base, USART_FLAG_TC)) {
            usart_flag_clear(usart->usart_base, USART_FLAG_TC);
        }
    }
}

/* Public functions ----------------------------------------------------------*/
#if CONFIG_USING_UART0

    void USART0_IRQHandler(void) {
        rt_interrupt_enter();
        usart_isr(&usart_cfg[IDX_USART0].serial);
        rt_interrupt_leave();
    }

#endif

rt_err_t bsp_hw_usart_init(void) {
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t ret;
    uint8_t i;

    ret = RT_ERROR;
    for (i = 0; i < NUMBER_OF_USART; i++) {
        usart_cfg[i].serial.ops = &usart_ops;
        usart_cfg[i].serial.config = cfg;
        /* register device */
        ret = rt_hw_serial_register(
            &usart_cfg[i].serial,
            usart_cfg[i].name,
            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
            &usart_cfg[i]);
        RT_ASSERT(ret == RT_EOK);
    }

    return ret;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(BOARD_SIPEED_LONGAN_NANO) && CONFIG_USING_DRIVER_SERIAL */

/******************** end of file *******************/
