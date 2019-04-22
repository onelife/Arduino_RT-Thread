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

#include "finsh.h"

#ifdef FINSH_USING_SYMTAB
struct finsh_syscall *_syscall_table_begin  = NULL;
struct finsh_syscall *_syscall_table_end    = NULL;
struct finsh_sysvar *_sysvar_table_begin    = NULL;
struct finsh_sysvar *_sysvar_table_end      = NULL;

#elif defined(CONFIG_ARDUINO)

# ifdef ADD_SHELL_CMD
#  undef ADD_SHELL_CMD
# endif

// insert function prototype
# define ADD_SHELL_CMD(prefix, name, desc, fn, r_type, ...) \
extern r_type fn(__VA_ARGS__);
# include "shell_cmd.h"
# undef ADD_SHELL_CMD

// insert function table entry
# ifdef FINSH_USING_DESCRIPTION
#  define ADD_SHELL_CMD(prefix, name, desc, fn, r_type, ...) \
    {prefix #name, #desc, (syscall_func)fn},
# else /* FINSH_USING_DESCRIPTION */
#  define ADD_SHELL_CMD(prefix, name, desc, fn, r_type, ...) \
    {prefix #name, (syscall_func)fn},
# endif /* FINSH_USING_DESCRIPTION */
struct finsh_syscall _syscall_table[] = {
    #include "shell_cmd.h"
};
# undef ADD_SHELL_CMD

# ifdef ADD_SHELL_VAR
#  undef ADD_SHELL_VAR
# endif

# if !defined(FINSH_USING_MSH_ONLY)
#  define finsh_type_unknown void*
#  define finsh_type_void void
#  define finsh_type_voidp void*
#  define finsh_type_char char
#  define finsh_type_uchar unsigned char
#  define finsh_type_charp char*
#  define finsh_type_short short
#  define finsh_type_ushort unsigned short
#  define finsh_type_shortp short*
#  define finsh_type_int int
#  define finsh_type_uint unsigned int
#  define finsh_type_intp int*
#  define finsh_type_long long
#  define finsh_type_ulong unsigned long
#  define finsh_type_longp unsigned*
#  define ADD_SHELL_VAR(name, desc, var, type) \
extern type var;

// insert variable prototype
#  include "shell_var.h"
#  undef finsh_type_unknown
#  undef finsh_type_void
#  undef finsh_type_voidp
#  undef finsh_type_char
#  undef finsh_type_uchar
#  undef finsh_type_charp
#  undef finsh_type_short
#  undef finsh_type_ushort
#  undef finsh_type_shortp
#  undef finsh_type_int
#  undef finsh_type_uint
#  undef finsh_type_intp
#  undef finsh_type_long
#  undef finsh_type_ulong
#  undef finsh_type_longp
#  undef ADD_SHELL_VAR

// insert variable table entry
#  ifdef FINSH_USING_DESCRIPTION
#   define ADD_SHELL_VAR(name, desc, var, type) \
    {#name, #desc, type, &var},
#  else
#   define ADD_SHELL_VAR(name, desc, var, type) \
    {#name, type, &var},
#  endif
struct finsh_sysvar _sysvar_table[] = {
    #include "shell_var.h"
};
#  undef ADD_SHELL_VAR
# endif /* !defined(FINSH_USING_MSH_ONLY) */

struct finsh_syscall *_syscall_table_begin = &_syscall_table[0];
struct finsh_syscall *_syscall_table_end   = \
    &_syscall_table[sizeof(_syscall_table) / sizeof(struct finsh_syscall)];
# if !defined(FINSH_USING_MSH_ONLY)
struct finsh_sysvar *_sysvar_table_begin  = &_sysvar_table[0];
struct finsh_sysvar *_sysvar_table_end    = \
    &_sysvar_table[sizeof(_sysvar_table) / sizeof(struct finsh_sysvar)];
# endif /* !defined(FINSH_USING_MSH_ONLY) */

#else /* FINSH_USING_SYMTAB */
long hello(void);
long version(void);
long list(void);
long list_thread(void);
long list_sem(void);
long list_mutex(void);
long list_fevent(void);
long list_event(void);
long list_mailbox(void);
long list_msgqueue(void);
long list_mempool(void);
long list_timer(void);

struct finsh_syscall _syscall_table[] =
{
    {"hello", hello},
    {"version", version},
    {"list", list},
    {"list_thread", list_thread},
#ifdef RT_USING_SEMAPHORE
    {"list_sem", list_sem},
#endif
#ifdef RT_USING_MUTEX
    {"list_mutex", list_mutex},
#endif
#ifdef RT_USING_FEVENT
    {"list_fevent", list_fevent},
#endif
#ifdef RT_USING_EVENT
    {"list_event", list_event},
#endif
#ifdef RT_USING_MAILBOX
    {"list_mb", list_mailbox},
#endif
#ifdef RT_USING_MESSAGEQUEUE
    {"list_mq", list_msgqueue},
#endif
#ifdef RT_USING_MEMPOOL
    {"list_memp", list_mempool},
#endif
    {"list_timer", list_timer},
};
struct finsh_syscall *_syscall_table_begin = &_syscall_table[0];
struct finsh_syscall *_syscall_table_end   = &_syscall_table[sizeof(_syscall_table) / sizeof(struct finsh_syscall)];
struct finsh_sysvar *_sysvar_table_begin  = NULL;
struct finsh_sysvar *_sysvar_table_end    = NULL;
#endif /* FINSH_USING_SYMTAB */

#endif /* RT_USING_FINSH */

