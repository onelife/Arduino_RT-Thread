/***************************************************************************//**
 * @file    mo_sym.h
 * @brief   Arduino RT-Thread module symbols
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#ifndef RT_USING_MODULE
# warning "Module symbols are not available for current CONFIG"
#endif

ADD_MODULE_SYM(__aeabi_idiv, int, int num, int denom)
ADD_MODULE_SYM(__aeabi_uidiv, unsigned, unsigned num, unsigned denom)
ADD_MODULE_SYM(__aeabi_idivmod, int, int numerator, int denominator)
ADD_MODULE_SYM(__aeabi_uidivmod, unsigned, unsigned numerator, unsigned denominator)

ADD_MODULE_SYM(dlopen, void *, const char *filename, int flags)
ADD_MODULE_SYM(dlclose, int, void *handle)
ADD_MODULE_SYM(dlsym, void *, void *handle, const char* symbol)
ADD_MODULE_SYM(dlerror, const char *, void)

ADD_MODULE_SYM(rt_tick_get, rt_tick_t, void)
ADD_MODULE_SYM(rt_tick_from_millisecond, int, rt_int32_t ms)

ADD_MODULE_SYM(rt_interrupt_enter, void, void)
ADD_MODULE_SYM(rt_interrupt_leave, void, void)
ADD_MODULE_SYM(rt_interrupt_get_nest, rt_uint8_t, void)

ADD_MODULE_SYM(rt_hw_interrupt_disable, rt_base_t, void)
ADD_MODULE_SYM(rt_hw_interrupt_enable, void, rt_base_t level)

ADD_MODULE_SYM(rt_object_get_information, struct rt_object_information *,
    enum rt_object_class_type type)

ADD_MODULE_SYM(rt_enter_critical, void, void)
ADD_MODULE_SYM(rt_critical_level, rt_uint16_t, void)

ADD_MODULE_SYM(rt_thread_init, rt_err_t, struct rt_thread *thread,
    const char *name, void (*entry)(void *parameter), void *parameter,
    void *stack_start, rt_uint32_t stack_size, rt_uint8_t priority,
    rt_uint32_t tick)
ADD_MODULE_SYM(rt_thread_self, rt_thread_t, void)
ADD_MODULE_SYM(rt_thread_startup, rt_err_t, rt_thread_t thread)
ADD_MODULE_SYM(rt_thread_detach, rt_err_t, rt_thread_t thread)
#ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_thread_create, rt_thread_t, const char *name,
    void (*entry)(void *parameter), void *parameter,
    rt_uint32_t stack_size, rt_uint8_t  priority, rt_uint32_t tick)
ADD_MODULE_SYM(rt_thread_delete, rt_err_t, rt_thread_t thread)
#endif
ADD_MODULE_SYM(rt_thread_yield, rt_err_t, void)
ADD_MODULE_SYM(rt_thread_delay, rt_err_t, rt_tick_t tick)
ADD_MODULE_SYM(rt_thread_mdelay, rt_err_t, rt_int32_t ms)
ADD_MODULE_SYM(rt_thread_control, rt_err_t, rt_thread_t thread, int cmd,
    void *arg)
ADD_MODULE_SYM(rt_thread_suspend, rt_err_t, rt_thread_t thread)
ADD_MODULE_SYM(rt_thread_resume, rt_err_t, rt_thread_t thread)
ADD_MODULE_SYM(rt_thread_timeout, void, void *parameter)
ADD_MODULE_SYM(rt_thread_find, rt_thread_t, char *name)
ADD_MODULE_SYM(rt_thread_sleep, rt_err_t, rt_tick_t tick)

ADD_MODULE_SYM(rt_timer_init, void, rt_timer_t timer, const char *name,
    void (*timeout)(void *parameter), void *parameter, rt_tick_t time,
    rt_uint8_t flag)
ADD_MODULE_SYM(rt_timer_detach, rt_err_t, rt_timer_t timer)
#ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_timer_create, rt_timer_t, const char *name,
    void (*timeout)(void *parameter), void *parameter, rt_tick_t time,
    rt_uint8_t flag)
ADD_MODULE_SYM(rt_timer_delete, rt_err_t, rt_timer_t timer)
#endif
ADD_MODULE_SYM(rt_timer_start, rt_err_t, rt_timer_t timer)
ADD_MODULE_SYM(rt_timer_stop, rt_err_t, rt_timer_t timer)
ADD_MODULE_SYM(rt_timer_control, rt_err_t, rt_timer_t timer, int cmd, void *arg)

ADD_MODULE_SYM(rt_get_errno, rt_err_t, void)
ADD_MODULE_SYM(rt_set_errno, void, rt_err_t no)
ADD_MODULE_SYM(_rt_errno, int *, void)
ADD_MODULE_SYM(rt_memset, void *, void *src, int c, rt_ubase_t n)
ADD_MODULE_SYM(rt_memcpy, void *, void *dest, const void *src, rt_ubase_t n)
ADD_MODULE_SYM(rt_memmove, void *, void *dest, const void *src, rt_ubase_t n)
ADD_MODULE_SYM(rt_memcmp, rt_int32_t, const void *cs, const void *ct,
    rt_ubase_t count)
ADD_MODULE_SYM(rt_strstr, char *, const char *str1, const char *str2)
ADD_MODULE_SYM(rt_strcasecmp, rt_uint32_t, const char *a, const char *b)
ADD_MODULE_SYM(rt_strncpy, char *, char *dest, const char *src, rt_ubase_t n)
ADD_MODULE_SYM(rt_strncmp, rt_int32_t, const char *cs, const char *ct,
    rt_ubase_t count)
ADD_MODULE_SYM(rt_strcmp, rt_int32_t, const char *cs, const char *ct)
ADD_MODULE_SYM(rt_strlen, rt_size_t, const char *src)
ADD_MODULE_SYM(rt_strdup, char *, const char *s)
ADD_MODULE_SYM(rt_show_version, void, void)
ADD_MODULE_SYM(rt_vsnprintf, rt_int32_t, char *buf, rt_size_t size,
    const char *fmt, va_list args)
ADD_MODULE_SYM(rt_snprintf, rt_int32_t, char *buf, rt_size_t size,
    const char *format, ...)
ADD_MODULE_SYM(rt_vsprintf, rt_int32_t, char *dest, const char *format,
    va_list arg_ptr)
ADD_MODULE_SYM(rt_sprintf, rt_int32_t, char *buf, const char *format, ...)
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
ADD_MODULE_SYM(rt_console_get_device, rt_device_t, void)
ADD_MODULE_SYM(rt_console_set_device, rt_device_t, const char *name)
#endif
ADD_MODULE_SYM(rt_hw_console_output, void, const char *str)
ADD_MODULE_SYM(rt_kprintf, void, const char *fmt, ...)
#ifdef RT_DEBUG
ADD_MODULE_SYM(rt_assert_handler, void, const char *ex_string, const char *func,
    rt_size_t line)
#endif

#ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_malloc_align, void *, rt_size_t size, rt_size_t align)
ADD_MODULE_SYM(rt_free_align, void, void *ptr)
# ifdef RT_USING_SMALL_MEM
ADD_MODULE_SYM(rt_malloc, void *, rt_size_t size)
ADD_MODULE_SYM(rt_realloc, void *, void *ptr, rt_size_t nbytes)
ADD_MODULE_SYM(rt_calloc, void *, rt_size_t count, rt_size_t size)
ADD_MODULE_SYM(rt_free, void, void *rmem)
# endif
# if defined(RT_USING_SLAB) && defined(RT_USING_HOOK)
ADD_MODULE_SYM(rt_malloc_sethook, void, void (*hook)(void *ptr, rt_size_t size))
ADD_MODULE_SYM(rt_free_sethook, void, void (*hook)(void *ptr))
# endif
#endif

#ifdef RT_USING_MEMHEAP
ADD_MODULE_SYM(rt_memheap_init, rt_err_t, struct rt_memheap *memheap,
    const char *name, void *start_addr, rt_size_t size)
ADD_MODULE_SYM(rt_memheap_detach, rt_err_t, struct rt_memheap *heap)
ADD_MODULE_SYM(rt_memheap_alloc, void *, struct rt_memheap *heap, rt_size_t size)
ADD_MODULE_SYM(rt_memheap_realloc, void *, struct rt_memheap *heap, void *ptr,
    rt_size_t newsize)
ADD_MODULE_SYM(rt_memheap_free, void, void *ptr)
#endif

#ifdef RT_USING_MEMPOOL
ADD_MODULE_SYM(rt_mp_init, rt_err_t, struct rt_mempool *mp, const char *name,
    void *start, rt_size_t size, rt_size_t block_size)
ADD_MODULE_SYM(rt_mp_detach, rt_err_t, struct rt_mempool *mp)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_mp_create, rt_mp_t, const char *name, rt_size_t block_count,
    rt_size_t block_size)
ADD_MODULE_SYM(rt_mp_delete, rt_err_t, rt_mp_t mp)
#endif
ADD_MODULE_SYM(rt_mp_alloc, void *, rt_mp_t mp, rt_int32_t time)
ADD_MODULE_SYM(rt_mp_free, void, void *block)
#endif

#ifdef RT_USING_MUTEX
ADD_MODULE_SYM(rt_mutex_init, rt_err_t, rt_mutex_t mutex, const char *name,
    rt_uint8_t flag)
ADD_MODULE_SYM(rt_mutex_detach, rt_err_t, rt_mutex_t mutex)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_mutex_create, rt_mutex_t, const char *name, rt_uint8_t flag)
ADD_MODULE_SYM(rt_mutex_delete, rt_err_t, rt_mutex_t mutex)
# endif
ADD_MODULE_SYM(rt_mutex_take, rt_err_t, rt_mutex_t mutex, rt_int32_t time)
ADD_MODULE_SYM(rt_mutex_release, rt_err_t, rt_mutex_t mutex)
ADD_MODULE_SYM(rt_mutex_control, rt_err_t, rt_mutex_t mutex, int cmd, void *arg)
#endif

#ifdef RT_USING_SEMAPHORE
ADD_MODULE_SYM(rt_sem_init, rt_err_t, rt_sem_t sem, const char *name,
    rt_uint32_t value, rt_uint8_t flag)
ADD_MODULE_SYM(rt_sem_detach, rt_err_t, rt_sem_t sem)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_sem_create, rt_sem_t, const char *name, rt_uint32_t value,
    rt_uint8_t flag)
ADD_MODULE_SYM(rt_sem_delete, rt_err_t, rt_sem_t sem)
# endif
ADD_MODULE_SYM(rt_sem_take, rt_err_t, rt_sem_t sem, rt_int32_t time)
ADD_MODULE_SYM(rt_sem_trytake, rt_err_t, rt_sem_t sem)
ADD_MODULE_SYM(rt_sem_release, rt_err_t, rt_sem_t sem)
ADD_MODULE_SYM(rt_sem_control, rt_err_t, rt_sem_t sem, int cmd, void *arg)
#endif

#ifdef RT_USING_EVENT
ADD_MODULE_SYM(rt_event_init, rt_err_t, rt_event_t event, const char *name,
    rt_uint8_t flag)
ADD_MODULE_SYM(rt_event_detach, rt_err_t, rt_event_t event)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_event_create, rt_event_t, const char *name, rt_uint8_t flag)
ADD_MODULE_SYM(rt_event_delete, rt_err_t, rt_event_t event)
# endif
ADD_MODULE_SYM(rt_event_send, rt_err_t, rt_event_t event, rt_uint32_t set)
ADD_MODULE_SYM(rt_event_recv, rt_err_t, rt_event_t event, rt_uint32_t set,
    rt_uint8_t opt, rt_int32_t timeout, rt_uint32_t *recved)
ADD_MODULE_SYM(rt_event_control, rt_err_t, rt_event_t event, int cmd, void *arg)
#endif

#ifdef RT_USING_MAILBOX
ADD_MODULE_SYM(rt_mb_init, rt_err_t, rt_mailbox_t mb, const char *name,
    void *msgpool, rt_size_t size, rt_uint8_t flag)
ADD_MODULE_SYM(rt_mb_detach, rt_err_t, rt_mailbox_t mb)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_mb_create, rt_mailbox_t, const char *name, rt_size_t size, rt_uint8_t flag)
ADD_MODULE_SYM(rt_mb_delete, rt_err_t, rt_mailbox_t mb)
# endif
ADD_MODULE_SYM(rt_mb_send, rt_err_t, rt_mailbox_t mb, rt_ubase_t value)
ADD_MODULE_SYM(rt_mb_send_wait, rt_err_t, rt_mailbox_t mb, rt_ubase_t value,
    rt_int32_t timeout)
ADD_MODULE_SYM(rt_mb_recv, rt_err_t, rt_mailbox_t mb, rt_ubase_t *value,
    rt_int32_t timeout)
ADD_MODULE_SYM(rt_mb_control, rt_err_t, rt_mailbox_t mb, int cmd, void *arg)
#endif

#ifdef RT_USING_MESSAGEQUEUE
ADD_MODULE_SYM(rt_mq_init, rt_err_t, rt_mq_t mq, const char *name,
    void *msgpool, rt_size_t msg_size, rt_size_t pool_size, rt_uint8_t flag)
ADD_MODULE_SYM(rt_mq_detach, rt_err_t, rt_mq_t mq)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_mq_create, rt_mq_t, const char *name, rt_size_t msg_size,
    rt_size_t max_msgs, rt_uint8_t flag)
ADD_MODULE_SYM(rt_mq_delete, rt_err_t, rt_mq_t mq)
# endif
ADD_MODULE_SYM(rt_mq_send, rt_err_t, rt_mq_t mq, void *buffer, rt_size_t size)
ADD_MODULE_SYM(rt_mq_urgent, rt_err_t, rt_mq_t mq, void *buffer, rt_size_t size)
ADD_MODULE_SYM(rt_mq_recv, rt_err_t, rt_mq_t mq, void *buffer, rt_size_t size,
    rt_int32_t timeout)
ADD_MODULE_SYM(rt_mq_control, rt_err_t, rt_mq_t mq, int cmd, void *arg)
#endif

#ifdef RT_USING_DEVICE
ADD_MODULE_SYM(rt_device_register, rt_err_t, rt_device_t dev, const char *name,
    rt_uint16_t flags)
ADD_MODULE_SYM(rt_device_unregister, rt_err_t, rt_device_t dev)
ADD_MODULE_SYM(rt_device_find, rt_device_t, const char *name)
# ifdef RT_USING_HEAP
ADD_MODULE_SYM(rt_device_create, rt_device_t, int type, int attach_size)
ADD_MODULE_SYM(rt_device_destroy, void, rt_device_t dev)
# endif
ADD_MODULE_SYM(rt_device_open, rt_err_t, rt_device_t dev, rt_uint16_t oflag)
ADD_MODULE_SYM(rt_device_close, rt_err_t, rt_device_t dev)
ADD_MODULE_SYM(rt_device_read, rt_size_t, rt_device_t dev, rt_off_t pos,
    void *buffer, rt_size_t size)
ADD_MODULE_SYM(rt_device_write, rt_size_t, rt_device_t dev, rt_off_t pos,
    const void *buffer, rt_size_t size)
ADD_MODULE_SYM(rt_device_control, rt_err_t, rt_device_t dev, int cmd, void *arg)
ADD_MODULE_SYM(rt_device_set_rx_indicate, rt_err_t, rt_device_t dev,
    rt_err_t (*rx_ind)(rt_device_t dev, rt_size_t size))
ADD_MODULE_SYM(rt_device_set_tx_complete, rt_err_t, rt_device_t dev,
    rt_err_t (*tx_done)(rt_device_t dev, void *buffer))
#endif

#ifdef RT_USING_DFS
ADD_MODULE_SYM(dfs_subdir, const char *, const char *directory,
    const char *filename)
ADD_MODULE_SYM(dfs_normalize_path, char *, const char *directory,
    const char *filename)
ADD_MODULE_SYM(open, int, const char *file, int flags, ...)
ADD_MODULE_SYM(close, int, int fd)
ADD_MODULE_SYM(read, int, int fd, void *buf, size_t len)
ADD_MODULE_SYM(write, int, int fd, const void *buf, size_t len)
ADD_MODULE_SYM(lseek, off_t, int fd, off_t offset, int whence)
ADD_MODULE_SYM(rename, int, const char *old, const char *new)
ADD_MODULE_SYM(unlink, int, const char *pathname)
ADD_MODULE_SYM(stat, int, const char *file, struct stat *buf)
ADD_MODULE_SYM(fstat, int, int fildes, struct stat *buf)
ADD_MODULE_SYM(fsync, int, int fildes)
ADD_MODULE_SYM(fcntl, int, int fildes, int cmd, ...)
ADD_MODULE_SYM(ioctl, int, int fildes, int cmd, ...)
ADD_MODULE_SYM(statfs, int, const char *path, struct statfs *buf)
ADD_MODULE_SYM(mkdir, int, const char *path, mode_t mode)
ADD_MODULE_SYM(rmdir, int, const char *pathname)
ADD_MODULE_SYM(opendir, DIR *, const char *name)
ADD_MODULE_SYM(readdir, struct dirent *, DIR *d)
ADD_MODULE_SYM(telldir, long, DIR *d)
ADD_MODULE_SYM(seekdir, void, DIR *d, off_t offset)
ADD_MODULE_SYM(rewinddir, void, DIR *d)
ADD_MODULE_SYM(closedir, int, DIR *d)
# ifdef DFS_USING_WORKDIR
ADD_MODULE_SYM(chdir, int, const char *path)
# endif
ADD_MODULE_SYM(getcwd, char *, char *buf, size_t size)
ADD_MODULE_SYM(system, int, const char *command)
#endif

#ifdef RT_USING_ULOG
ADD_MODULE_SYM(ulog_output, void, rt_uint32_t level,
    const char *tag, rt_bool_t newline, const char *format, ...)
#endif


/* Please add your commands with the following format:

 ADD_MODULE_SYM(function_name, return_type, parameter list)
 - function_name: must be a valid C identifier 
 - return_type: must be a valid C variable type
 - parameter list: list of parameters (with types) separated by ","; void for empty list

 Example: void *rt_malloc(rt_size_t size); =>
 ADD_MODULE_SYM(rt_malloc, void *, rt_size_t size)
 */

