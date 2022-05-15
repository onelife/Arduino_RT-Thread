/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2010-03-22     Bernard      first version
 */
 
#include "include/rtthread.h"

#ifdef RT_USING_FINSH

#include "components/finsh/finsh.h"

#if defined(CONFIG_ARDUINO)

# ifdef ADD_SHELL_CMD
#  undef ADD_SHELL_CMD
# endif

    // extern const syscall_func __fsym_##fn##_ptr = fn;
// insert function prototype
# ifdef FINSH_USING_DESCRIPTION
#  define ADD_SHELL_CMD(fn) \
    extern long fn(void); \
    extern const char __fsym_##fn##_name[]; \
    extern const char __fsym_##fn##_desc[];
# else /* FINSH_USING_DESCRIPTION */
#  define ADD_SHELL_CMD(fn) \
    extern long fn(void); \
    extern const char __fsym_##fn##_name[];
# endif /* FINSH_USING_DESCRIPTION */
# include "shell_cmd.h"
# undef ADD_SHELL_CMD

// insert function table entry
# ifdef FINSH_USING_DESCRIPTION
#  define ADD_SHELL_CMD(fn) \
    {__fsym_##fn##_name, __fsym_##fn##_desc, (syscall_func)fn},
# else /* FINSH_USING_DESCRIPTION */
#  define ADD_SHELL_CMD(fn) \
    {__fsym_##fn##_name, (syscall_func)#fn},
# endif /* FINSH_USING_DESCRIPTION */
static struct finsh_syscall _syscall_table[] = {
    #include "shell_cmd.h"
};
# undef ADD_SHELL_CMD

struct finsh_syscall *_syscall_table_begin = &_syscall_table[0];
struct finsh_syscall *_syscall_table_end   = \
    &_syscall_table[sizeof(_syscall_table) / sizeof(struct finsh_syscall)];

#endif /* CONFIG_ARDUINO */

#endif /* RT_USING_FINSH */

