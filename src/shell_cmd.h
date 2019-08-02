/***************************************************************************//**
 * @file    shell_cmd.h
 * @brief   Arduino RT-Thread shell commands
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifdef FINSH_USING_MSH_ONLY
# define ADD_FINSH_CMD(name, desc, fn, r_type, ...)
#else
# define ADD_FINSH_CMD(...) ADD_SHELL_CMD("", __VA_ARGS__)
#endif
#ifdef FINSH_USING_MSH
# define ADD_MSH_CMD(...) ADD_SHELL_CMD("__cmd_", __VA_ARGS__)
#else
# define ADD_MSH_CMD(name, desc, fn, r_type, ...)
#endif

ADD_FINSH_CMD(list, list available commands, list, void, void)
ADD_FINSH_CMD(hello, say hello world, hello, void, void)
ADD_FINSH_CMD(version, show RT-Thread version info, version, void, void)
ADD_FINSH_CMD(list_thread, list threads, list_thread, void, void)
ADD_MSH_CMD(help, RT-Thread shell help, msh_help, int, int argc, char **argv)
ADD_MSH_CMD(ver, show RT-Thread version info, version, void, void)
ADD_MSH_CMD(ps, list threads, cmd_ps, int, int argc, char **argv)
#ifdef RT_USING_HEAP
ADD_FINSH_CMD(list_mem, show memory usage info, list_mem, void, void)
ADD_MSH_CMD(free, show memory usage info, cmd_free, int, int argc, char **argv)
# ifdef RT_USING_MEMTRACE
ADD_MSH_CMD(mcheck, check memory, memcheck, int, void)
ADD_MSH_CMD(mtrace, show memory trace info, memtrace, int, int argc, char **argv)
# endif
#endif
#ifdef RT_USING_SEMAPHORE
ADD_FINSH_CMD(list_sem, list semaphores in system, list_sem, void, void)
ADD_MSH_CMD(lsem, list semaphores in system, list_sem, int, int argc, char **argv)
#endif
#ifdef RT_USING_MUTEX
ADD_FINSH_CMD(list_mutex, list mutex in system, list_mutex, void, void)
ADD_MSH_CMD(lmtx, list mutex in system, list_mutex, void, void)
#endif
#ifdef RT_USING_EVENT
ADD_FINSH_CMD(list_event, list event in system, list_event, void, void)
ADD_MSH_CMD(levt, list event in system, list_event, void, void)
#endif
#ifdef RT_USING_MAILBOX
ADD_FINSH_CMD(list_mb, list mail box in system, list_mailbox, void, void)
ADD_MSH_CMD(lmb, list mail box in system, list_mailbox, void, void)
#endif
#ifdef RT_USING_MESSAGEQUEUE
ADD_FINSH_CMD(list_mq, list message queue in system, list_msgqueue, void, void)
ADD_MSH_CMD(lmq, list message queue in system, list_msgqueue, void, void)
#endif
#ifdef RT_USING_MEMPOOL
ADD_FINSH_CMD(list_memp, list memory pool in system, list_mempool, void, void)
ADD_MSH_CMD(lmp, list memory pool in system, list_mempool, void, void)
#endif
#ifdef RT_USING_MEMHEAP
ADD_FINSH_CMD(list_memheap, list memory heap in system, list_mempool, long, void)
ADD_MSH_CMD(lmh, list memory heap in system, list_mempool, long, void)
#endif
ADD_FINSH_CMD(list_timer, list timer in system, list_timer, void, void)
ADD_MSH_CMD(ltmr, list timer in system, list_timer, void, void)
#ifdef RT_USING_DEVICE
ADD_FINSH_CMD(list_dev, list device in system, list_device, void, void)
ADD_MSH_CMD(ldev, list device in system, list_device, void, void)
#endif
#ifdef RT_USING_RTC
ADD_FINSH_CMD(list_date, show date and time, list_date, void, void)
ADD_FINSH_CMD(set_date, set date, set_date, rt_err_t, rt_uint32_t, rt_uint32_t, rt_uint32_t)
ADD_FINSH_CMD(set_time, set time, set_time, rt_err_t, rt_uint32_t, rt_uint32_t, rt_uint32_t)
ADD_MSH_CMD(date, access date and time, date, void, int argc, char **argv)
#endif /* RT_USING_RTC */
#ifdef RT_USING_DFS
ADD_FINSH_CMD(df, show free space info of disk, df, int, const char *path)
ADD_FINSH_CMD(list_fd, list file descriptors, list_fd, int, void)
ADD_FINSH_CMD(mkfs, format disk with file system, mkfs, void, const char *fs_name, const char *device_name)
ADD_FINSH_CMD(mkdir, create a directory, mkdir, int, const char *path, mode_t mode)
ADD_FINSH_CMD(ls, list directory contents, ls, void, char *pathname)
ADD_FINSH_CMD(rm, remove files or directories, rm, void, const char *filename)
ADD_FINSH_CMD(copy, copy file or dir, copy, void, const char *src, const char *dst)
ADD_FINSH_CMD(cat, print file content, cat, void, const char *filename)
ADD_MSH_CMD(df, show free space info of disk, cmd_df, int, int argc, char **argv)
ADD_MSH_CMD(lfd, list file descriptors, list_fd, int, void)
ADD_MSH_CMD(mkfs, format disk with file system, cmd_mkfs, int, int argc, char **argv)
ADD_MSH_CMD(mkdir, create a directory, cmd_mkdir, int, int argc, char **argv)
ADD_MSH_CMD(ls, list directory contents, cmd_ls, int, int argc, char **argv)
ADD_MSH_CMD(mv, move or rename file, cmd_mv, int, int argc, char **argv)
ADD_MSH_CMD(rm, remove files or directories, cmd_rm, int, int argc, char **argv)
ADD_MSH_CMD(cp, copy file or dir, cmd_cp, int, int argc, char **argv)
ADD_MSH_CMD(cat, print file content, cmd_cat, int, int argc, char **argv)
ADD_MSH_CMD(echo, insert string to file, cmd_echo, int, int argc, char **argv)
# ifdef DFS_USING_WORKDIR
ADD_FINSH_CMD(cd, change current working directory, chdir, int, const char *path)
ADD_MSH_CMD(cd, change current working directory, cmd_cd, int, int argc, char **argv)
ADD_MSH_CMD(pwd, show current working directory, cmd_pwd, int, int argc, char **argv)
# endif
#endif /* RT_USING_DFS */
#if CONFIG_USING_SPISD
ADD_FINSH_CMD(list_sd, show SD info, list_sd, rt_err_t, void)
ADD_MSH_CMD(lsd, show SD info, list_sd, rt_err_t, void)
#endif
#ifdef RT_USING_MODULE
ADD_MSH_CMD(lsym, list symbols info, list_symbols, int, void)
ADD_MSH_CMD(lmod, list modules in system, list_module, int, void)
#endif
#if CONFIG_USING_GUI
# ifdef RT_USING_DFS
ADD_FINSH_CMD(screenshot, capture screen, screenshot, void, const char *filename)
ADD_MSH_CMD(prtscn, capture screen, prtscn, int, int argc, char **argv)
# endif
#endif


/* Please add your commands with the following format:

 ADD_FINSH_CMD(command_name, command description, function_name, return_type, parameter list)
 - command_name (alias of function_name): must be a valid C identifier 
 - command description: may have space
 - function_name: must be a valid C identifier 
 - return_type: must be a valid C variable type
 - parameter list: list of parameters (with types) separated by ","; void for empty list

 Example 1: void led(rt_int32_t val); =>
 ADD_FINSH_CMD(led0, Turn on/off builtin LED, led, void, rt_int32_t val)

 Example 2: void led_on(void); =>
 ADD_FINSH_CMD(led0_on, Turn on builtin LED, led_on, void, void)

 Example 3: rt_uint32_t led_set(rt_uint32_t id, rt_int32_t val); =>
 ADD_FINSH_CMD(led, Turn on/off any LED, led_set, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
 */
// ADD_FINSH_CMD(led, Turn on/off builtin LED, led, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
// ADD_MSH_CMD(led, Turn on/off builtin LED, led, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
