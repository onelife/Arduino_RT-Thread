/***************************************************************************//**
 * @file    drv_spisd.cpp
 * @brief   Arduino RT-Thread library SPI SD driver
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
extern "C" {

#include "include/rtthread.h"

#if defined(CONFIG_ARDUINO) && CONFIG_USING_SPISD
}

#include <Arduino.h>
#include <SPI.h>    /* Arduino library */

extern "C" {

#include "drv_spi.h"
#include "drv_spisd.h"

/***************************************************************************//**
 * @addtogroup Arduino
 * @{
 ******************************************************************************/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef BSP_SD_DEBUG
#define sd_debug(format, args...)       rt_kprintf(format, ##args)
#else
#define sd_debug(format, args...)
#endif
#define SD_CS_PIN                       CONFIG_SD_CS_PIN
#define SD_CS_INIT()                    pinMode(SD_CS_PIN, OUTPUT); \
                                        digitalWrite(SD_CS_PIN, HIGH)
#define SD_CS(en)                       digitalWrite(SD_CS_PIN, en ? LOW : HIGH)
#define _NUM_TO_STR(n)                  #n
#define _CH_TO_STR(ch)                  _NUM_TO_STR(ch)
#define SD_LOWER_DEVICE_NAME            "SPI" _CH_TO_STR(CONFIG_SD_SPI_CHANNEL)
#define SD_CTX()                        (&sd_ctx)
#define SD_RESTART_TIMER(ctx)           (ctx->tout) = RT_FALSE; \
                                        rt_timer_start(&ctx->tmr)
#define SD_STOP_TIMER(ctx)              rt_timer_stop(&ctx->tmr)
#define SD_IS_TIMEOUT(ctx)              (ctx->tout)

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct bsp_sd_contex sd_ctx;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief - send SD command
 *
 * @param[in] struct bsp_sd_contex *ctx - pointer to SD contex
 *
 * @param[in] rt_uint8_t cmd = command index
 *
 * @param[in] rt_uint32_t arg - command argument
 *
 * @param[in] rt_uint8_t *trail - pointer to trailing data buffer
 *
 * @return rt_uint16_t - command response
 *
 ******************************************************************************/
static rt_uint16_t _sd_send_cmd(struct bsp_sd_contex *ctx, rt_uint8_t cmd,
    rt_uint32_t arg, rt_uint8_t *trail) {
    rt_uint8_t buf_ins[RT_ALIGN(11, RT_ALIGN_SIZE)];
    rt_uint8_t buf_res[32];
    rt_uint16_t ret = 0xffff;

    /* Expect (x+1+4) bytes for CRC, (x+1+19) bytes for CSD/CID */
    rt_memset(buf_res, 0xff, sizeof(buf_res));
    sd_debug("SPISD: send cmd %d (%08x)\n", cmd, arg);

    do {
        rt_size_t read_len;
        rt_uint8_t len_trl, i, j;
        rt_bool_t skip;

        /*  build cmd inst
            - inst len: 6
            - cmd index: 1-byte
            - cmd param: 4-byte (MSB first)
            - rx buf addr: offset align with RT_ALIGN_SIZE
        */
        buf_ins[0] = 6;
        buf_ins[1] = 0x40 | cmd;
        buf_ins[2] = (arg >> 24) & 0x000000ff;
        buf_ins[3] = (arg >> 16) & 0x000000ff;
        buf_ins[4] = (arg >> 8) & 0x000000ff;
        buf_ins[5] = arg & 0x000000ff;
        switch (cmd) {
        case CMD0:  buf_ins[6] = 0x95; break;   /* valid CRC for CMD0(0x0) */
        case CMD8:  buf_ins[6] = 0x87; break;   /* valid CRC for CMD8(0x1AA) */
        case CMD58: buf_ins[6] = 0x01; break;   /* dummy CRC + stop */
        default:    buf_ins[6] = 0x01; break;   /* dummy CRC + stop */
        }
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        *(rt_uint8_t **)(&buf_ins[RT_ALIGN(7, RT_ALIGN_SIZE)]) = buf_res;
        #pragma GCC diagnostic pop

        /* set trail length */
        switch (cmd) {
        case CMD8:  len_trl = 4; break;                 /* R7 response */
        case CMD9:  len_trl = SD_BLOCK_SIZE_CSD; break; 
        case CMD10: len_trl = SD_BLOCK_SIZE_CID; break;
        case CMD58: len_trl = SD_BLOCK_SIZE_OCR; break; /* R3 response */
        default:    len_trl = 0; break;
        }

        /* send cmd and get reply */
        read_len = rt_device_read(ctx->ldev, 0, buf_ins, sizeof(buf_res));
        if (0 == read_len) {
            #ifdef BSP_SD_DEBUG
            rt_uint8_t *tmp = buf_res;
            rt_uint32_t j;
            for (j = 0; j < sizeof(buf_res); j += 8)
                sd_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                    tmp[j], tmp[j+1], tmp[j+2], tmp[j+3],
                    tmp[j+4], tmp[j+5], tmp[j+6], tmp[j+7]);
            #endif
            sd_debug("SPISD err: send cmd failed! [%d]\n", read_len);
            break;
        }

        /* skip one byte when stop reading */
        skip = (cmd == CMD12) ? RT_TRUE : RT_FALSE;

        /* find valid response: the response is read back within command
           response time (NCR), 0 to 8 bytes for SDC, 1 to 8 bytes for MMC */
        for (i = 0; i < sizeof(buf_res); i++) {
            if (0xff != buf_res[i]) {
                if (skip) {
                    sd_debug("SPISD: skip %02x (@ %d)\n", buf_res[i], i);
                    skip = RT_FALSE;
                    continue;
                }
                if (cmd == (ACMD13 & 0x7f))
                    ret = (rt_uint16_t)buf_res[i];  /* R2 response */
                else
                    ret = (rt_uint8_t)buf_res[i];
                break;
            }
        }
        sd_debug("SPISD: response %02x (@ %d)\n", ret, i);
        i++;
        
        /* copy trailing data */
        if ((0xffff != ret) && len_trl && trail) {
            /* read CSD/CID */
            if ((CMD9 == cmd) || (CMD10 == cmd)) {
                /* find data block */
                for (; i < sizeof(buf_res); i++)
                    if (buf_res[i] == 0xfe) break;
                /* check if valid */
                if (sizeof(buf_res) <= i) {
                    sd_debug("SPISD err: no CSD/CID!\n");
                    ret = 0xffff;
                    break;
                }
                i++;
                sd_debug("SPISD: CSD/CID %02x (@ %d)\n", buf_res[i], i);
                sd_debug("SPISD: read CRC %02x %02x\n", buf_res[i+len_trl],
                    buf_res[i+len_trl+1]);
            }
            /* copy data */
            for (j = 0; j < len_trl; j++)
                trail[j] = buf_res[i + j];
        }
    } while (0);

    return ret;
}

rt_uint16_t sd_send_cmd(struct bsp_sd_contex *ctx, rt_uint8_t cmd,
    rt_uint32_t arg, rt_uint8_t *trail) {
    /* ACMD<n> = CMD55 + CMD<n> */
    if (cmd & 0x80) {
        rt_uint16_t ret = _sd_send_cmd(ctx, CMD55, 0x00000000, RT_NULL);
        if (0x01 < ret) return ret;
        cmd &= 0x7f;
    }

    return _sd_send_cmd(ctx, cmd, arg, trail);
}

/***************************************************************************//**
 * @brief - read a block of data from SD card
 *
 * @note - the function handles responses of command ACMD13, CMD17 and CMD18
 *
 * @param[in] struct bsp_sd_contex *ctx - poniter to SD context
 *
 * @param[in] void *buf - poniter to response buffer
 *
 * @param[in] rt_size_t size - buffer size in byte
 *
 * @return rt_err_t - error code
 *
 ******************************************************************************/
static rt_err_t sd_read_block(struct bsp_sd_contex *ctx, void *buf,
    rt_size_t size) {
    rt_uint8_t buf_ins[RT_ALIGN(5, RT_ALIGN_SIZE)];
    rt_uint8_t buf_res[8];
    rt_err_t ret;

    sd_debug("SPISD: read block [%d]\n", size);
    do {
        rt_size_t read_len;

        /*  build read data inst
            - inst len: 0
            - rx buf addr: offset align with RT_ALIGN_SIZE
         */
        buf_ins[0] = 0;
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        *(rt_uint8_t **)(&buf_ins[RT_ALIGN(1, RT_ALIGN_SIZE)]) = \
            (rt_uint8_t *)buf;
        #pragma GCC diagnostic pop

        /* read with token (starting indicator) 0xfe */
        read_len = rt_device_read(ctx->ldev, 0xfe, buf_ins, size);
        if (0 == read_len) {
            sd_debug("SPISD err: read data failed!\n");
            ret = -RT_EIO;
            break;
        }

        /*  build read CRC inst
            - inst len: 0
            - rx buf addr: offset align with RT_ALIGN_SIZE
         */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        *(rt_uint8_t **)(&buf_ins[RT_ALIGN(1, RT_ALIGN_SIZE)]) = buf_res;
        #pragma GCC diagnostic pop

        read_len = rt_device_read(ctx->ldev, 0, buf_ins, sizeof(buf_res));
        if (0 == read_len) {
            sd_debug("SPISD err: read CRC failed!\n");
            ret = -RT_EIO;
            break;
        }
        sd_debug("SPISD: read CRC %x %x\n", buf_res[0], buf_res[1]);

        ret = RT_EOK;
    } while(0);

    if (RT_EOK != ret)
        sd_debug("SPISD err: read block failed! [%02x]\n", ret);
    return ret;
}

/***************************************************************************//**
 * @brief - read raw data from SD device
 *
 * @param[in] struct bsp_sd_contex *ctx - pointer to SD contex
 *
 * @param[in] void *buf - poniter to read buffer
 *
 * @param[in] rt_size_t size - buffer size in byte
 *
 * @return rt_size_t - number of bytes read
 *
 ******************************************************************************/
static rt_size_t sd_read(struct bsp_sd_contex *ctx, void *buf, rt_size_t size) {
    rt_uint8_t buf_read[RT_ALIGN(5, RT_ALIGN_SIZE)];
    rt_size_t ret;

    /*  build read data inst
        - inst len: 0
        - rx buf addr: offset align with RT_ALIGN_SIZE
     */
    buf_read[0] = 0x00;
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"
    *(rt_uint8_t **)(&buf_read[RT_ALIGN(1, RT_ALIGN_SIZE)]) = (rt_uint8_t *)buf; 
    #pragma GCC diagnostic pop

    ret = rt_device_read(ctx->ldev, 0, buf_read, size);
    if (0 == ret) {
        #ifdef BSP_SD_DEBUG
        rt_uint8_t *tmp = (rt_uint8_t *)buf;
        rt_uint32_t j;
        for (j = 0; j < size; j += 8)
            sd_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                tmp[j], tmp[j+1], tmp[j+2], tmp[j+3],
                tmp[j+4], tmp[j+5], tmp[j+6], tmp[j+7]);
        #endif
        sd_debug("SPISD err: read raw failed! [%d]\n", ret);
    }

    return ret;
}

/***************************************************************************//**
 * @brief - write a block of data to SD card
 *
 * @note - the function sends data and control tokens for block write command
 *   CMD24 and CMD25
 *
 * @param[in] struct bsp_sd_contex *ctx - Poniter to SD context
 *
 * @param[in] void *buf - Poniter to command buffer
 *
 * @param[in] rt_uint8_t token - Control token
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t sd_write_block(struct bsp_sd_contex *ctx, void *buf,
    rt_uint8_t token) {
    rt_uint8_t buf_ins[RT_ALIGN(11, RT_ALIGN_SIZE)];
    rt_uint8_t buf_res[8];
    rt_err_t ret;

    sd_debug("SPISD: write block [%02x]\n", token);
    do {
        rt_size_t read_len, write_len;
        rt_uint8_t i;

        /* waiting for SD ready */
        SD_RESTART_TIMER(ctx);
        do {
            sd_read(ctx, buf_res, sizeof(buf_res));
        } while (!SD_IS_TIMEOUT(ctx) && (0xff != buf_res[sizeof(buf_res) - 1]));
        SD_STOP_TIMER(ctx);

        if (0xff != buf_res[sizeof(buf_res) - 1]) {
            sd_debug("SPISD err: SD busy b/f write! [%02x]\n", \
                buf_res[sizeof(buf_res) - 1]);
            ret = -RT_EBUSY;
            break;
        }

        if (0xfd != token) {
            /* send data */
            /*  build write token + data inst
                - inst len: 1
                - token (inst): 1-byte
                - tx buf addr: offset align with RT_ALIGN_SIZE
             */
            buf_ins[0] = 1;
            buf_ins[1] = token;
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(rt_uint8_t **)(&buf_ins[RT_ALIGN(2, RT_ALIGN_SIZE)]) = \
                (rt_uint8_t *)buf;
            #pragma GCC diagnostic pop

            write_len = rt_device_write(ctx->ldev, 0, buf_ins, SD_SECTOR_SIZE);
            if (0 == write_len) {
                sd_debug("SPISD err: write data failed!\n");
                ret = -RT_EIO;
                break;
            }

            /*  build read crc inst
                - inst len: 0
                - rx buf addr: offset align with RT_ALIGN_SIZE
             */
            buf_ins[0] = 0;
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(rt_uint8_t **)(&buf_ins[RT_ALIGN(1, RT_ALIGN_SIZE)]) = buf_res;
            #pragma GCC diagnostic pop

            read_len = rt_device_read(ctx->ldev, 0, buf_ins, sizeof(buf_res));
            if (0 == read_len) {
                sd_debug("SPISD err: write CRC failed!\n");
                ret = -RT_EIO;
                break;
            }

            /* check if write inst accepted */
            for (i = 0; i < sizeof(buf_res); i++) {
                if (buf_res[i] != 0xff) {
                    buf_res[i] &= 0x1f;
                    break;
                }
            }
            if (0x05 != buf_res[i]) {
                sd_debug("SPISD err: write is not accepted! (%02x @ %d)\n", \
                    buf_res[i], i);
                ret = -RT_EIO;
                break;
            }

        } else {
            /* send token */
            /*  build write token + data inst
                - inst len: 1
                - token (inst): 1-byte
                - tx buf addr: offset align with RT_ALIGN_SIZE
             */
            buf_ins[0] = 1;                             /* inst length */
            buf_ins[1] = token;
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Wstrict-aliasing"
            *(rt_uint8_t **)(&buf_ins[RT_ALIGN(2, RT_ALIGN_SIZE)]) = RT_NULL;
            #pragma GCC diagnostic pop

            write_len = rt_device_write(ctx->ldev, 0, buf_ins, 0);
            if (0 != write_len) {
                sd_debug("SPISD err: write token failed!\n");
                ret = -RT_EIO;
                break;
            }

            /* waiting for SD ready */
            SD_RESTART_TIMER(ctx);
            do {
                sd_read(ctx, buf_res, sizeof(buf_res));
            } while (!SD_IS_TIMEOUT(ctx) && (0xff != buf_res[sizeof(buf_res) - 1]));
            SD_STOP_TIMER(ctx);

            if (0xff != buf_res[sizeof(buf_res) - 1]) {
                sd_debug("SPISD err: SD busy a/f write! [%02x]\n",
                    buf_res[sizeof(buf_res) - 1]);
                ret = -RT_EBUSY;
                break;
            }
        }

        ret = RT_EOK;
    } while (0);

    if (RT_EOK != ret)
        sd_debug("SPISD err: write block failed! [%02x]\n", ret);
    return ret;
}

static rt_err_t bsp_spiSd_init(rt_device_t dev) {
    struct bsp_sd_contex *ctx = (struct bsp_sd_contex *)(dev->user_data);
    rt_err_t ret;

    /* open lower level device */
    ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != ret) return ret;

    do {
        rt_uint8_t tril[4];
        rt_uint8_t type = 0;
        rt_uint8_t retry = SD_INIT_RETRY;

        /* switch to low speed */
        ret = rt_device_control(ctx->ldev, RT_DEVICE_CTRL_SPI_SPEED,
            (void *)SD_SPEED_LOW);
        if (RT_EOK != ret) break;

        /* provide >74 dummy clocks */
        (void)rt_device_read(ctx->ldev, 0, RT_NULL, 10);

        /* enter idle state */
        SD_CS(1);
        while (retry && (0x01 != sd_send_cmd(ctx, CMD0, 0x00000000, RT_NULL)))
            retry--;
        if (!retry) {
            ret = -RT_EIO;
            break;
        }

        /* check if SDv2 */
        if (0x01 == sd_send_cmd(ctx, CMD8, 0x000001AA, tril)) {
            /* SDv2, Vdd: 2.7-3.6V */
            if (((tril[2] & 0x0F) == 0x01) && (tril[3] == 0xAA)) {
                SD_RESTART_TIMER(ctx);
                /* SD init (ACMD41 with HCS bit) */
                while (!SD_IS_TIMEOUT(ctx) && \
                    sd_send_cmd(ctx, ACMD41, 0x40000000, RT_NULL));
                /* check CCS bit (bit 30) in the OCR */
                if (!SD_IS_TIMEOUT(ctx) && \
                    (0x00 ==sd_send_cmd(ctx, CMD58, 0x00000000, tril)))
                    type = (tril[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
            }
        } else {
            SD_RESTART_TIMER(ctx);
            /* SD init */
            while (!SD_IS_TIMEOUT(ctx) && \
                sd_send_cmd(ctx, ACMD41, 0x00000000, RT_NULL));
            if (!SD_IS_TIMEOUT(ctx)) {
                /* SDv1 */
                type = CT_SD1;
            } else {
                SD_RESTART_TIMER(ctx);
                /* MMC init */
                while (!SD_IS_TIMEOUT(ctx) && \
                    sd_send_cmd(ctx, CMD1, 0x00000000, RT_NULL));
                if (!SD_IS_TIMEOUT(ctx)) {
                    /* MMCv3 */
                    type = CT_MMC;
                }
            }
        }
        SD_STOP_TIMER(ctx);

        /* set read/write block length to 512 bytes */
        if ((type > 0) && !(type | CT_BLOCK))
            if (sd_send_cmd(ctx, CMD16, 0x00000200, RT_NULL) != 0x00)
                type = 0;

        /* check type */
        if (type) {
            /* init ok */
            ret = rt_device_control(ctx->ldev, RT_DEVICE_CTRL_SPI_SPEED,
                (void *)SD_SPEED_HIGH);
            if (RT_EOK != ret) break;
        } else {
            ret = -RT_EIO;
            break;
        }
        ctx->type = type;

        sd_debug("SPISD: init ok, card type %x\n", ctx->type);
    } while (0);

    SD_CS(0);
    rt_device_close(ctx->ldev);

    if (RT_EOK != ret)
        sd_debug("SPISD err: init failed! (%08x)\n", ret);
    return ret;
}

static rt_size_t bsp_spiSd_read(rt_device_t dev, rt_off_t sector, void *buf,
    rt_size_t count) {
    struct bsp_sd_contex *ctx = (struct bsp_sd_contex *)(dev->user_data);
    rt_err_t err;
    rt_size_t ret;

    /* open SPI device */
    err = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != err) {
        rt_set_errno(err);
        return 0;
    };
    sd_debug("SPISD: read sect %d [%d]\n", sector, count);

    /* convert to byte address if necessary */
    if (!(ctx->type & CT_BLOCK))
        sector *= SD_SECTOR_SIZE;

    do {
        rt_size_t cnt = count;
        rt_uint8_t *ptr = (rt_uint8_t *)buf;
        rt_uint8_t cmd;

        ret = 0;
        SD_CS(1);

        if (1 == cnt) {
            /* single block read */
            cmd = CMD17;
            sd_debug("SPISD: read single block\n");
        } else {
            /* multiple block read */
            cmd = CMD18;
            sd_debug("SPISD: read multiple blocks\n");
        }

        if (sd_send_cmd(ctx, cmd, sector, RT_NULL)) {
            sd_debug("SPISD err: read cmd failed!\n");
            err = -RT_EIO;
            break;
        }

        /* read data */
        do {
            if (RT_EOK != (err = sd_read_block(ctx, ptr, SD_SECTOR_SIZE)))
                break;
            ptr += SD_SECTOR_SIZE;
        } while (--cnt);

        /* stop transmission */
        if (CMD18 == cmd)
            if (sd_send_cmd(ctx, CMD12, 0x00000000, RT_NULL)) {
                err = -RT_EOK;
                break;
            }

        ret = count;
    } while (0);

    SD_CS(0);
    rt_device_close(ctx->ldev);
    if (RT_EOK != err) {
        rt_set_errno(err);
        sd_debug("SPISD err: read failed! [%08x]\n", err);
    }

    sd_debug("SPISD: read ok! [%d]\n", ret);
    return ret;
}

static rt_size_t bsp_spiSd_write(rt_device_t dev, rt_off_t sector,
    const void *buf, rt_size_t count) {
    struct bsp_sd_contex *ctx = (struct bsp_sd_contex *)(dev->user_data);
    rt_err_t err;
    rt_size_t ret;

    /* open SPI device */
    err = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != err) {
        rt_set_errno(err);
        return 0;
    };
    sd_debug("SPISD: write sect %d [%d]\n", sector, count);

    /* convert to byte address if needed */
    if (!(ctx->type & CT_BLOCK))
        sector *= SD_SECTOR_SIZE;

    do {
        rt_size_t cnt = count;
        rt_uint8_t *ptr = (rt_uint8_t *)buf;
        rt_uint8_t cmd, token;

        ret = 0;
        SD_CS(1);

        if (1 == cnt) {
            /* single block write */
            cmd = CMD24;
            token = 0xfe;
            sd_debug("SPISD: write single block\n");
        } else {
            /* multiple block write */
            cmd = CMD25;
            token = 0xfc;
            sd_debug("SPISD: write multiple blocks\n");
            if (ctx->type & CT_SDC)
                if (sd_send_cmd(ctx, ACMD23, count, RT_NULL)) {
                    err = -RT_EOK;
                    break;
                }
        }

        if (sd_send_cmd(ctx, cmd, sector, RT_NULL)) {
            sd_debug("SPISD err: write command error!\n");
            err = -RT_EOK;
            break;
        }

        /* write data */
        do {
            if (sd_write_block(ctx, ptr, token)) {
                err = -RT_EOK;
                break;
            }
            ptr += SD_SECTOR_SIZE;
        } while (--cnt);

        /* stop transmission token */
        if (sd_write_block(ctx, RT_NULL, 0xfd)) {
            err = -RT_EOK;
            break;
        }

        ret = count;
    } while (0);

    SD_CS(0);
    rt_device_close(ctx->ldev);
    if (RT_EOK != err) {
        rt_set_errno(err);
        sd_debug("SPISD err: write failed! [%08x]\n", err);
    }

    sd_debug("SPISD: write ok! [%d]\n", ret);
    return ret;
}

static rt_err_t bsp_spiSd_control(rt_device_t dev, rt_int32_t cmd, void *buf) {
    struct bsp_sd_contex *ctx = (struct bsp_sd_contex *)(dev->user_data);
    rt_uint8_t *buf_res = RT_NULL;
    rt_err_t ret;

    sd_debug("SPISD: control %08x\n", cmd);
    switch (cmd) {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        ret = rt_device_open(ctx->ldev, RT_DEVICE_OFLAG_RDWR);
        if (RT_EOK != ret) break;
        sd_debug("SPISD: control open\n");

        do {
            struct rt_device_blk_geometry *geometry = \
                (struct rt_device_blk_geometry *)buf;
            rt_uint32_t c_size;
            rt_uint8_t n;
            
            /* allocate buf */
            buf_res = (rt_uint8_t *)rt_malloc(SD_BLOCK_SIZE_CSD);
            if (RT_NULL == buf_res) {
                sd_debug("SPISD: no memory for RX buf\n");
                ret = -RT_ENOMEM;
                break;
            }

            SD_CS(1);
            /* get number of sectors on the disk (32 bits) */
            if (sd_send_cmd(ctx, CMD9, 0x00000000, buf_res)) {
                sd_debug("SPISD: get CSD failed!\n");
                ret = -RT_EIO;
                break;
            }

            if (0x01 == (buf_res[0] >> 6)) {
                /* SDv2 */
                /* C_SIZE: Bit 48~69 */
                c_size = ((rt_uint32_t)(buf_res[7] & 0x3f) << 16) + \
                         ((rt_uint32_t)buf_res[8] << 8) + buf_res[9] + 1;
                /* result = capacity / sector_size */
                geometry->sector_count = c_size << (19 - SD_SECTOR_SIZE_SHIFT);
            } else {
                /* SDv1 or MMC */
                /* C_SIZE: Bit 62~73 */
                c_size = ((rt_uint32_t)(buf_res[6] & 0x03) << 10) + \
                         ((rt_uint16_t)buf_res[7] << 2) + (buf_res[8] >> 6) + 1;
                /* READ_BL_LEN: Bit 80~83, C_SIZE_MULT: Bit 47~49 */
                n = ((buf_res[9] & 0x03) << 1) + ((buf_res[10] & 0x80) >> 7) + \
                    2 + (buf_res[5] & 0x0f);
                /* result = capacity / sector_size */
                geometry->sector_count = c_size << (n - SD_SECTOR_SIZE_SHIFT);
            }
            rt_free(buf_res);

            /* get sector size */
            geometry->bytes_per_sector = SD_SECTOR_SIZE;
            
            /* get erase block size in unit of sectors (32 bits) */
            if (ctx->type & CT_SD2) {
                /* allocate buf */
                buf_res = (rt_uint8_t *)rt_malloc(SD_BLOCK_SIZE_SDSTAT);
                if (RT_NULL == buf_res) {
                    sd_debug("SPISD: no memory for RX buf\n");
                    ret = -RT_ENOMEM;
                    break;
                }
                /* SDv2 */
                if (sd_send_cmd(ctx, ACMD13, 0x00000000, RT_NULL)) {
                    sd_debug("SPISD: get SD status failed!\n");
                    ret = -RT_EIO;
                    break;
                }
                if (sd_read_block(ctx, buf_res, SD_BLOCK_SIZE_SDSTAT)) {
                    sd_debug("SPISD: read SD status failed!\n");
                    ret = -RT_EIO;
                    break;
                }
                /* AU_SIZE: Bit 428~431 */
                geometry->block_size = \
                    16UL << ((buf_res[10] >> 4) + 9 - SD_SECTOR_SIZE_SHIFT);
            } else {
                /* allocate buf */
                buf_res = (rt_uint8_t *)rt_malloc(SD_BLOCK_SIZE_CSD);
                if (RT_NULL == buf_res) {
                    sd_debug("SPISD: no memory for RX buf\n");
                    ret = -RT_ENOMEM;
                    break;
                }
                /* SDv1 or MMC */
                if (sd_send_cmd(ctx, CMD9, 0x00000000, buf_res)) {
                    sd_debug("SPISD: Get CSD failed!\n");
                    ret = -RT_EIO;
                    break;
                }

                if (ctx->type & CT_SD1)  {
                    /* SECTOR_SIZE: Bit 39~45, WRITE_BL_LEN: Bit 22~25 (9, 10 or 11) */
                    geometry->block_size = \
                        (((buf_res[10] & 0x3f) << 1) + \
                        ((rt_uint32_t)(buf_res[11] & 0x80) >> 7) + 1) << \
                        (8 + (buf_res[13] >> 6) - SD_SECTOR_SIZE_SHIFT);
                } else {
                    /* ERASE_GRP_SIZE: Bit 42~46, ERASE_GRP_MULT: Bit 37~41 */
                    geometry->block_size = \
                        ((rt_uint16_t)((buf_res[10] & 0x7c) >> 2) + 1) * \
                        (((buf_res[10] & 0x03) << 3) + \
                        ((buf_res[11] & 0xe0) >> 5) + 1);
                }
            }

        } while (0);

        SD_CS(0);
        rt_device_close(ctx->ldev);
        if (buf_res) rt_free(buf_res);
        break;

    case RT_DEVICE_CTRL_BLK_SYNC:
        /* flush dirty buf if present */
        ret = RT_EOK;
        break;

    case RT_DEVICE_CTRL_BLK_ERASE:
    case RT_DEVICE_CTRL_BLK_AUTOREFRESH:
    default:
        ret = -RT_EINVAL;
        break;
    }

    if (RT_EOK != ret)
        sd_debug("SPISD err: control failed! [%08x]\n", ret);

    return ret;
}

static void sd_timer(void *param) {
    struct bsp_sd_contex *ctx = (struct bsp_sd_contex *)param;
    ctx->tout = RT_TRUE;
}

/***************************************************************************//**
 * @brief Initialize SD contex
 *
 * @param[in] struct bsp_sd_contex *ctx - Pointer to SD contex
 *
 * @param[in] const char *name - Pointer to SD name
 *
 * @param[in] void *ldev - Pointer to lower level device
 *
 * @return rt_err_t - Error code
 *
 ******************************************************************************/
static rt_err_t bsp_spiSd_contex_init(struct bsp_sd_contex *ctx,
    const char *name, rt_device_t ldev) {
    rt_err_t ret;

    do {
        ctx->type = 0;
        ctx->ldev = ldev;
        ctx->tout = RT_FALSE;

        /* init timer */
        rt_timer_init(&ctx->tmr, name, sd_timer, (void *)ctx,
            SD_TIMER_DELAY, RT_TIMER_FLAG_ONE_SHOT);

        /* register device */
        ctx->dev.type = RT_Device_Class_MTD;
        ctx->dev.rx_indicate = RT_NULL;
        ctx->dev.tx_complete = RT_NULL;
        ctx->dev.init = bsp_spiSd_init;
        ctx->dev.open = RT_NULL;
        ctx->dev.close = RT_NULL;
        ctx->dev.read = bsp_spiSd_read;
        ctx->dev.write = bsp_spiSd_write;
        ctx->dev.control = bsp_spiSd_control;
        ctx->dev.user_data = (void *)ctx;
        ret = rt_device_register(&ctx->dev, name, RT_DEVICE_FLAG_RDWR | \
            RT_DEVICE_FLAG_REMOVABLE);
    } while (0);

    return ret;
}

/* Public functions ----------------------------------------------------------*/
/***************************************************************************//**
 * @brief - Initialize SD contex and hardware
 *
 * @return rt_err_t - error code
 *
 ******************************************************************************/
rt_err_t bsp_hw_spiSd_init(void) {
    rt_err_t ret;

    do {
        rt_device_t ldev;

        /* get lower level device */
        ldev = rt_device_find(SD_LOWER_DEVICE_NAME);
        if (RT_NULL == ldev) {
            sd_debug("SPISD: can't find device %s!\n", SD_LOWER_DEVICE_NAME);
            ret = -RT_ERROR;
            break;
        }
        sd_debug("SPISD: find device %s\n", SD_LOWER_DEVICE_NAME);

        ret = bsp_spiSd_contex_init(SD_CTX(), SD_NAME, ldev);
        if (RT_EOK != ret) break;

        SD_CS_INIT();

        sd_debug("SPISD: h/w init ok\n");
    } while (0);

    if (RT_EOK != ret)
        rt_kprintf("SPISD: h/w init failed!\n");

    return ret;
}

/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
#if 0   //#ifdef RT_USING_FINSH
#include "components/finsh/finsh.h"

rt_err_t list_sd(void) {
    struct bsp_sd_contex *ctx = SD_CTX();
    rt_uint8_t buf_res[16];
    struct sd_register_cid *cid = (sd_register_cid *)buf_res;
    struct rt_device_blk_geometry geometry;
    rt_uint32_t temp;
    rt_err_t ret;

    if (RT_EOK != (ret = rt_device_open(&ctx->dev, RT_DEVICE_OFLAG_RDWR))) {
        rt_kprintf("Error: open failed!\n");
        return ret;
    }
    SD_CS(1);

    /* Receive CID as a data block (16 bytes) */
    if (sd_send_cmd(ctx, CMD10, 0x00000000, buf_res)) {
        SD_CS(0);
        rt_device_close(&ctx->dev);
        rt_kprintf("Error: Get CID failed!\n");
        return -RT_ERROR;
    }
    SD_CS(0);

    rt_kprintf("    SD Card on %s\n", ctx->ldev->parent.name);
    rt_kprintf(" ------------------------------\n");
    rt_kprintf(" Manufacturer ID:\t%X\n", cid->man_id);
    rt_kprintf(" OEM/Application ID:\t%c%c\n", cid->app_id[0], cid->app_id[1]);
    buf_res[13] = 0;
    rt_kprintf(" Product name:\t\t%c%c%c%c%c\n", cid->name[0], cid->name[1],
        cid->name[2], cid->name[3], cid->name[4]);
    rt_kprintf(" Product revision:\t%X.%X\n", (cid->rev & 0xF0) >> 4,
        (cid->rev & 0x0F));
    temp = ((rt_uint32_t)cid->sn[0] << 24) + ((rt_uint32_t)cid->sn[1] << 16) + \
        ((rt_uint32_t)cid->sn[2] << 8) + (rt_uint32_t)cid->sn[3];
    rt_kprintf(" Serial number:\t\t%X\n", temp);
    rt_kprintf(" Manufacturing date:\t%d.%d\n", \
        2000 + ((cid->date[0] & 0x0F) * 10) + ((cid->date[1] & 0xF0) >> 4), \
        cid->date[1] & 0x0F);

    rt_kprintf(" Card type:\t\t");
    if (ctx->type & CT_MMC)
        rt_kprintf("%s\n", "MMC");
    else if (ctx->type & CT_SDC)
        rt_kprintf("%s\n", "SDXC");
    else if (ctx->type & CT_SD1)
        rt_kprintf("%s\n", "SDSC");
    else if (ctx->type & CT_SD2)
        rt_kprintf("%s\n", "SDHC");
    else
        rt_kprintf("0x%X\n", ctx->type);

    rt_device_control(&ctx->dev, RT_DEVICE_CTRL_BLK_GETGEOME, &geometry);
    rt_device_close(&ctx->dev);

    temp = ((geometry.sector_count & 0x0000FFFF) * \
           geometry.bytes_per_sector) >> 16;
    temp += ((geometry.sector_count >> 16) * geometry.bytes_per_sector);
    temp >>= 4;
    rt_kprintf(" Card capacity:\t\t%dMB\n", temp);

    return RT_EOK;
}
FINSH_FUNCTION_EXPORT(list_sd, list the SD card.)
#endif /* RT_USING_FINSH */

/***************************************************************************//**
 * @}
 ******************************************************************************/

#endif /* defined(CONFIG_ARDUINO) && CONFIG_USING_SPISD */

} /* extern "C" */
