/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Change Logs:
 * Date           Author      Notes
 * 2018/08/29     Bernard     first version
 * 2019/04/25     onelife     refactor
 */

#include "include/rtthread.h"

#ifdef RT_USING_MODULE
#ifdef __arm__

#include "../dlelf.h"

#ifdef RT_USING_ULOG
# define LOG_LVL                    LOG_LVL_INFO
# define LOG_TAG                    "MO_ARM"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_D                      LOG_E
#endif /* RT_USING_ULOG */


Elf32_Addr dlmodule_relocate(rt_uint8_t rel_type, Elf32_Addr *where,
    Elf32_Addr sym_val) {
    /*  Tested rel_type:
        - R_ARM_JUMP_SLOT
        - R_ARM_RELATIVE
     */
    Elf32_Addr tmp;
    Elf32_Sword addend, offset;
    rt_uint32_t upper, lower, sign, j1, j2;
    union {
        Elf32_Word word;
        Elf32_Half half[2];
    } tmp2;

    switch (rel_type) {
    case R_ARM_NONE:
        LOG_E("R_ARM_NONE");
        return 0;

    case R_ARM_GLOB_DAT:
    case R_ARM_JUMP_SLOT:
        /* (S + A) | T */
        return sym_val;

    case R_ARM_RELATIVE:
        /* B(S) + A */
        return sym_val + *where;

    case R_ARM_ABS32:
        LOG_I("R_ARM_ABS32");
        return sym_val + *where;

    case R_ARM_PC24:
    case R_ARM_PLT32:
    case R_ARM_CALL:
    case R_ARM_JUMP24:
        LOG_I("R_ARM_CALL");
        addend = *where & 0x00ffffff;
        if (addend & 0x00800000) {
            addend |= 0xff000000;
        }
        tmp = sym_val - (Elf32_Addr)where + (addend << 2);
        tmp >>= 2;
        return (*where & 0xff000000) | (tmp & 0x00ffffff);

    case R_ARM_REL32:
        LOG_I("R_ARM_REL32");
        return sym_val - (Elf32_Addr)where + *where;

    case R_ARM_V4BX:
        LOG_I("R_ARM_V4BX");
        return ((Elf32_Addr)where & 0xf000000f) | 0x01a0f000;

    #if 0 /* todo */
    case R_ARM_GOT_BREL:
        LOG_I("R_ARM_GOT_BREL");
        temp = sym_val;
        return (Elf32_Addr)&temp;
    #endif

    case R_ARM_THM_CALL:
    case R_ARM_THM_JUMP24:
        LOG_I("R_ARM_THM_CALL");
        upper  = *(rt_uint16_t *)where;
        lower  = *(rt_uint16_t *)((Elf32_Addr)where + 2);
        sign   = (upper >> 10) & 1;
        j1     = (lower >> 13) & 1;
        j2     = (lower >> 11) & 1;
        offset = (sign << 24) | \
                 ((~(j1 ^ sign) & 1) << 23) | \
                 ((~(j2 ^ sign) & 1) << 22) | \
                 ((upper & 0x03ff) << 12) | \
                 ((lower & 0x07ff) << 1);
        if (offset & 0x01000000) {
            offset -= 0x02000000;
        }
        offset += sym_val - (Elf32_Addr)where;

        if (!(offset & 1) || offset <= (rt_int32_t)0xff000000 || \
                             offset >= (rt_int32_t)0x01000000) {
            LOG_E("R_ARM_THM_CALL: bad Thumb sym_val");
            return -RT_ERROR;
        }

        sign = (offset >> 24) & 1;
        j1   = sign ^ (~(offset >> 23) & 1);
        j2   = sign ^ (~(offset >> 22) & 1);
        tmp2.half[0] = (rt_uint16_t)((upper & 0xf800) | \
                                     (sign << 10) | \
                                     ((offset >> 12) & 0x03ff));
        tmp2.half[1] = (rt_uint16_t)((lower & 0xd000) | \
                                     (j1 << 13) | (j2 << 11) | \
                                     ((offset >> 1) & 0x07ff));
        return tmp2.word;

    default:
        LOG_E("bad rel_type %d", rel_type);
        return 0;
    }
}

#endif /* __arm__ */
#endif /* RT_USING_MODULE */
