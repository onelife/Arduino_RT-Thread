/***************************************************************************//**
 * @file    dfs_patch.h
 * @brief   Arduino RT-Thread library ChaN's FatFs patch header
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef __DFS_PATCH_H__
#define __DFS_PATCH_H__

FRESULT f_seekdir(DIR *dj, int offset);                             /* Seek in directory */

#endif /* __DFS_PATCH_H__ */
