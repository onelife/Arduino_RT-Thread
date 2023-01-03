/***************************************************************************//**
 * @file    shell_cmd.h
 * @brief   Arduino RT-Thread shell commands
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
# define ADD_MSH_CMD ADD_SHELL_CMD

ADD_MSH_CMD(msh_help)
ADD_MSH_CMD(clear)
ADD_MSH_CMD(version)
ADD_MSH_CMD(list_thread)
#ifdef MSH_USING_BUILT_IN_COMMANDS
ADD_MSH_CMD(cmd_ps)
# ifdef RT_USING_HEAP
ADD_MSH_CMD(cmd_free)
# endif
#endif
#ifdef RT_USING_SEMAPHORE
ADD_MSH_CMD(list_sem)
#endif
#ifdef RT_USING_EVENT
ADD_MSH_CMD(list_event)
#endif
#ifdef RT_USING_MUTEX
ADD_MSH_CMD(list_mutex)
#endif
#ifdef RT_USING_MAILBOX
ADD_MSH_CMD(list_mailbox)
#endif
#ifdef RT_USING_MESSAGEQUEUE
ADD_MSH_CMD(list_msgqueue)
#endif
#ifdef RT_USING_MEMHEAP
ADD_MSH_CMD(list_memheap)
#endif
#ifdef RT_USING_MEMPOOL
ADD_MSH_CMD(list_mempool)
#endif
ADD_MSH_CMD(list_timer)
#ifdef RT_USING_DEVICE
ADD_MSH_CMD(list_device)
#endif
#ifdef RT_USING_DFS
ADD_MSH_CMD(list_fd)
# ifdef DFS_USING_POSIX
ADD_MSH_CMD(cmd_ls)
ADD_MSH_CMD(cmd_cp)
ADD_MSH_CMD(cmd_mv)
ADD_MSH_CMD(cmd_cat)
ADD_MSH_CMD(cmd_rm)
#  ifdef DFS_USING_WORKDIR
ADD_MSH_CMD(cmd_cd)
ADD_MSH_CMD(cmd_pwd)
#  endif
ADD_MSH_CMD(cmd_mkdir)
ADD_MSH_CMD(cmd_mkfs)
ADD_MSH_CMD(cmd_mount)
ADD_MSH_CMD(cmd_umount)
ADD_MSH_CMD(cmd_df)
ADD_MSH_CMD(cmd_echo)
ADD_MSH_CMD(cmd_tail)
# endif /* DFS_USING_POSIX */
#endif /* RT_USING_DFS */
#ifdef RT_USING_SMALL_MEM
# ifdef RT_USING_MEMTRACE
ADD_MSH_CMD(memcheck)
ADD_MSH_CMD(memtrace)
# endif
#endif
#ifdef RT_USING_MEMHEAP
# ifdef RT_USING_MEMTRACE
ADD_MSH_CMD(memheapcheck)
ADD_MSH_CMD(memheaptrace)
# endif
#endif


// #ifdef RT_USING_RTC
// ADD_MSH_CMD(date, access date and time, date, void, int argc, char **argv)
// #endif /* RT_USING_RTC */
#if !CONFIG_USING_DRIVER_SPI && CONFIG_USING_SPISD
ADD_MSH_CMD(list_sd)
#endif
#ifdef RT_USING_MODULE
ADD_MSH_CMD(list_symbols)
ADD_MSH_CMD(list_module)
#endif
#if CONFIG_USING_GUI
# ifdef RT_USING_DFS
ADD_MSH_CMD(prtscn)
# endif
#endif

// #if defined __has_include
// # if __has_include("user_cmd.h")
// #  include "user_cmd.h"
// # endif
// #endif


/* NOTES
    Please use "MSH_CMD_EXPORT_ALIAS" macro to export your command with the following format:

    MSH_CMD_EXPORT_ALIAS(function_name, command_name, command description)
     - function_name: must be a valid C identifier
     - command_name (alias of function_name): must be a valid C identifier 
     - command description: may have space

    And then append the command with the following format:

     ADD_MSH_CMD(function_name)
     - function_name: The same function_name as used for "MSH_CMD_EXPORT_ALIAS"

     Example: int led_set(int argc, char **argv); =>
     MSH_CMD_EXPORT_ALIAS(led_set, toggle_led, Turn on/off builtin LED.);
     ADD_MSH_CMD(led_set)
 */
// ADD_MSH_CMD(led_set)
