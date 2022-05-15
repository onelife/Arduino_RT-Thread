/***************************************************************************//**
 * @file    dfs_patch.h
 * @brief   Arduino RT-Thread library ChaN's FatFs patch header
 * @note    This file includes modifications made on original FatFs code to make
 *            maintenance easier
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include "include/rtthread.h"


FRESULT f_seekdir(
    DIR *dj,        /* Pointer to the open directory object */
    int offset      /* the seek offset */
)
{
    int i = 0;

    if (dir_sdi(dj, 0) != FR_OK || offset < 0)
        return FR_INT_ERR;

    while(i < offset)
    {
        if(dir_read(dj, 0) != FR_OK || dir_next(dj, 0) != FR_OK)
            return FR_INT_ERR;
        i++;
    }
    return FR_OK;
}


#if FF_VOLUMES > 1

int elm_get_vol(FATFS *fat)
{
    int vol;

    for (vol = 0; vol < FF_VOLUMES; vol ++)
    {
        if (FatFs[vol] == fat) return vol;
    }

    return -1;
}

#endif  /* FF_VOLUMES > 1 */
