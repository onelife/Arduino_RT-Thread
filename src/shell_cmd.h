/***************************************************************************//**
 * @file    shell_cmd.h
 * @brief   Arduino RT-Thread shell commands
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
ADD_SHELL_CMD(hello, say hello world, hello, void, void)
ADD_SHELL_CMD(version, show RT-Thread version information, version, void, void)
ADD_SHELL_CMD(list, list available commands, list, void, void)
ADD_SHELL_CMD(list_mem, list memory usage information, list_mem, void, void)
ADD_SHELL_CMD(list_thread, list thread, list_thread, void, void)
#ifdef RT_USING_SEMAPHORE
ADD_SHELL_CMD(list_sem, list semaphore in system, list_sem, void, void)
#endif
#ifdef RT_USING_MUTEX
ADD_SHELL_CMD(list_mutex, list mutex in system, list_mutex, void, void)
#endif
#ifdef RT_USING_EVENT
ADD_SHELL_CMD(list_event, list event in system, list_event, void, void)
#endif
#ifdef RT_USING_MAILBOX
ADD_SHELL_CMD(list_mb, list mail box in system, list_mailbox, void, void)
#endif
#ifdef RT_USING_MESSAGEQUEUE
ADD_SHELL_CMD(list_mq, list message queue in system, list_msgqueue, void, void)
#endif
#ifdef RT_USING_MEMPOOL
ADD_SHELL_CMD(list_memp, list memory pool in system, list_mempool, void, void)
#endif
#ifdef RT_USING_MEMHEAP
ADD_SHELL_CMD(list_memheap, list memory heap in system, list_mempool, long, void)
#endif
ADD_SHELL_CMD(list_timer, list timer in system, list_timer, void, void)


/* Please add your commands with the following format:

 ADD_SHELL_CMD(command_name, command description, function_name, return_type, parameter list)
 - command_name (alias of function_name): must be a valid C identifier 
 - command description: may have space
 - function_name: must be a valid C identifier 
 - return_type: must be a valid C variable type
 - parameter list: list of parameters (with types) separated by ","; void for empty list

 Example 1: void led(rt_int32_t val); =>
 ADD_SHELL_CMD(led0, Turn on/off builtin LED, led, void, rt_int32_t val)

 Example 2: void led_on(void); =>
 ADD_SHELL_CMD(led0_on, Turn on builtin LED, led_on, void, void)

 Example 3: long led_set(rt_uint32_t id, rt_int32_t val); =>
 ADD_SHELL_CMD(led, Turn on/off any LED, led_set, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
 */
// ADD_SHELL_CMD(led, Turn on/off builtin LED, led, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
