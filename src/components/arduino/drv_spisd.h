/***************************************************************************//**
 * @file    drv_spisd.h
 * @brief   Arduino RT-Thread library SPI SD driver header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DRV_SPISD_H__
#define __DRV_SPISD_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define SD_NAME                 "SD"
#define SD_SPEED_LOW            (250000)
#define SD_SPEED_HIGH           (SPI_MAX_SPEED)

#define SD_INIT_RETRY           (6)
#define SD_TIMER_DELAY          (2 * RT_TICK_PER_SECOND)

#define SD_SECTOR_SIZE_SHIFT    (9)         /* 512 bytes is always supported */
#define SD_SECTOR_SIZE          (1 << SD_SECTOR_SIZE_SHIFT)
#define SD_BLOCK_SIZE_CSD       (16)
#define SD_BLOCK_SIZE_CID       (16)
#define SD_BLOCK_SIZE_OCR       (4)
#define SD_BLOCK_SIZE_SDSTAT    (64)

/* Card type definitions (CardType) */
#define CT_MMC                  (0x01)
#define CT_SD1                  (0x02)
#define CT_SD2                  (0x04)
#define CT_SDC                  (CT_SD1 | CT_SD2)
#define CT_BLOCK                (0x08)

/* Definitions for MMC/SDC command */
#define CMD0                    (0)         /* GO_IDLE_STATE */
#define CMD1                    (1)         /* SEND_OP_COND */
#define ACMD41                  (41|0x80)   /* SEND_OP_COND (SDC) */
#define CMD8                    (8)         /* SEND_IF_COND */
#define CMD9                    (9)         /* SEND_CSD */
#define CMD10                   (10)        /* SEND_CID */
#define CMD12                   (12)        /* STOP_TRANSMISSION */
#define ACMD13                  (13|0x80)   /* SD_STATUS (SDC) */
#define CMD16                   (16)        /* SET_BLOCKLEN */
#define CMD17                   (17)        /* READ_SINGLE_BLOCK */
#define CMD18                   (18)        /* READ_MULTIPLE_BLOCK */
#define CMD23                   (23)        /* SET_BLOCK_COUNT */
#define ACMD23                  (23|0x80)   /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24                   (24)        /* WRITE_BLOCK */
#define CMD25                   (25)        /* WRITE_MULTIPLE_BLOCK */
#define CMD41                   (41)        /* SEND_OP_COND (ACMD) */
#define CMD55                   (55)        /* APP_CMD */
#define CMD58                   (58)        /* READ_OCR */

/* Exported types ------------------------------------------------------------*/
struct bsp_sd_contex {
    rt_uint16_t type;       /* card type */
    rt_bool_t tout;         /* timeout indicator */
    rt_device_t ldev;       /* lower level device (RT SPI) */
    struct rt_timer tmr;    /* timer */
    struct rt_device dev;   /* RT device */
};

struct sd_register_cid {
    rt_uint8_t man_id;
    rt_uint8_t app_id[2];
    rt_uint8_t name[5];
    rt_uint8_t rev;
    rt_uint8_t sn[4];
    rt_uint8_t date[2];
    rt_uint8_t crc;
};

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_spiSd_init(void);

#endif /* __DRV_SPISD_H__ */
