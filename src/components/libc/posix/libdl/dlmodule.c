/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author      Notes
 * 2018/08/29     Bernard     first version
 * 2019/04/25     onelife     refactor, reduce ram usage, bug fix
 */

#include "include/rtthread.h"
#ifdef RT_USING_MODULE

#include "include/rthw.h"
#include "dlfcn.h"
#include "dlmodule.h"
#include "dlelf.h"
// #include "components/dfs/include/dfs_posix.h"
#include "components/dfs/include/dfs_file.h"

#ifdef RT_USING_ULOG
# define LOG_LVL                    LOG_LVL_INFO
# define LOG_TAG                    "MO_DLM"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# define LOG_I                      LOG_E
# define LOG_D                      LOG_E
#endif /* RT_USING_ULOG */

#ifndef MODULE_THREAD_PRIORITY
# define MODULE_THREAD_PRIORITY     (RT_THREAD_PRIORITY_MAX - 1)
#endif
#ifndef MODULE_THREAD_STACK_SIZE
# define MODULE_THREAD_STACK_SIZE   (2 * 1024)
#endif
#ifndef MODULE_THREAD_TICK
# define MODULE_THREAD_TICK         (10)
#endif
#define MODULE_MIN_STACK_SIZE       (1 * 1024)
#define MODULE_MAX_STACK_SIZE       (10 * 1024)

#define BREAK_WITH_WARN(err, msg, args...) {\
    LOG_W(msg, ##args); \
    ret = -err; \
    break; \
}

#ifdef CONFIG_ARDUINO
# ifdef ADD_MODULE_SYM
#  undef ADD_MODULE_SYM
# endif

// insert function prototype
# define ADD_MODULE_SYM(fn, r_type, ...) extern r_type fn(__VA_ARGS__);
# include "mo_sym.h"
# undef ADD_MODULE_SYM

// insert function table entry
# define ADD_MODULE_SYM(fn, r_type, ...) {(void *)&fn, #fn},
static struct rt_module_symtab _rt_module_symtab[] = {
    ADD_MODULE_SYM(dlmodule_find, struct rt_dlmodule, const char *name)
    #include "mo_sym.h"
};
# undef ADD_MODULE_SYM

static struct rt_module_symtab *_rt_module_symtab_begin = &_rt_module_symtab[0];
static struct rt_module_symtab *_rt_module_symtab_end = &_rt_module_symtab[\
    sizeof(_rt_module_symtab) / sizeof(struct rt_module_symtab)];

#else /* CONFIG_ARDUINO */
static struct rt_module_symtab *_rt_module_symtab_begin = RT_NULL;
static struct rt_module_symtab *_rt_module_symtab_end   = RT_NULL;

# ifdef __IAR_SYSTEMS_ICC__
#  pragma section="RTMSymTab"
# endif
#endif /* CONFIG_ARDUINO */


/* set the name of module */
static void _dlmodule_set_name(struct rt_dlmodule *module, const char *path) {
    int size;
    struct rt_object *object;
    const char *first, *end, *ptr;

    object = &(module->parent);
    ptr = first = (char *)path;
    end = path + rt_strlen(path);

    while (*ptr != '\0') {
        if (*ptr == '/') first = ptr + 1;
        if (*ptr == '.') end = ptr - 1;
        ptr++;
    }

    size = end - first + 1;
    if (size > RT_NAME_MAX) size = RT_NAME_MAX;

    rt_strncpy(object->name, first, size);
    object->name[size] = '\0';
}

#define RT_MODULE_ARG_MAX 8
static int _rt_module_split_arg(char *cmd, rt_size_t length, char *argv[]) {
    int argc = 0;
    char *ptr = cmd;

    while ((rt_size_t)(ptr - cmd) < length) {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && \
               (rt_size_t)(ptr - cmd) < length) {
            *ptr++ = '\0';
        }
        /* check whether it's the end of line */
        if ((rt_size_t)(ptr - cmd) >= length) break;

        /* handle string with quote */
        if (*ptr == '"') {
            argv[argc++] = ++ptr;
            /* skip this string */
            while (*ptr != '"' && (rt_size_t)(ptr - cmd) < length) {
                if (*ptr++ == '\\') ptr++;
            }
            if ((rt_size_t)(ptr - cmd) >= length) break;
            /* skip '"' */
            *ptr++ = '\0';
        } else {
            argv[argc++] = ptr;
            while ((*ptr != ' ' && *ptr != '\t') && \
                   (rt_size_t)(ptr - cmd) < length) {
                ptr++;
            }
        }

        if (argc >= RT_MODULE_ARG_MAX) break;
    }

    return argc;
}

/* invoked by main thread for exit */
static void _dlmodule_exit(void) {
    struct rt_dlmodule *module;

    module = dlmodule_self();
    if (!module) return; /* not a module thread */

    rt_enter_critical();
    if (RT_DLMODULE_STAT_RUNNING == module->stat) {
        struct rt_object    *object = RT_NULL;
        struct rt_list_node *node = RT_NULL;

        /* set stat to closing */
        module->stat = RT_DLMODULE_STAT_CLOSING;

        /* suspend all threads in this module */
        for (node = module->object_list.next;
             node != &(module->object_list);
             node = node->next) {
            object = rt_list_entry(node, struct rt_object, list);

            if (RT_Object_Class_Thread == \
                (object->type & ~RT_Object_Class_Static)) {
                rt_thread_t thread = (rt_thread_t)object;

                /* stop timer and suspend thread*/
                if (RT_THREAD_CLOSE != (thread->stat & RT_THREAD_STAT_MASK) ||
                    RT_THREAD_INIT  != (thread->stat & RT_THREAD_STAT_MASK)) {
                    rt_timer_stop(&(thread->thread_timer));
                    rt_thread_suspend(thread);
                }
            }
        }
    }
    rt_exit_critical();

    LOG_I("mo exit");
    return;
}

static void _dlmodule_thread_entry(void* parameter) {
    int argc;
    char *argv[RT_MODULE_ARG_MAX];
    struct rt_dlmodule *module;

    do {
        module = (struct rt_dlmodule *)parameter;

        if ((RT_NULL == module) || (RT_NULL == module->cmd_line)) break;

        rt_memset(argv, 0x00, sizeof(argv));
        argc = _rt_module_split_arg(
            (char *)module->cmd_line, rt_strlen(module->cmd_line), argv);
        if (!argc) break;

        /* set status of module */
        LOG_I("mo %s entry @%p", module->parent.name, module->entry_addr);
        module->stat = RT_DLMODULE_STAT_RUNNING;

        if (module->entry_addr) {
            module->entry_addr(argc, argv);
        }
    } while (0);

    /* done execution */
    _dlmodule_exit();
}

struct rt_dlmodule *dlmodule_create(void) {
    struct rt_dlmodule *module = RT_NULL;

    module = (struct rt_dlmodule *)rt_object_allocate(
        RT_Object_Class_Module, "module");
    if (!module) return RT_NULL;

    module->stat = RT_DLMODULE_STAT_INIT;
    module->priority = MODULE_THREAD_PRIORITY;
    module->stack_size = MODULE_THREAD_STACK_SIZE;
    rt_list_init(&(module->object_list));
    LOG_D("mo %s create", module->parent.name);

    return module;
}

void dlmodule_destroy_subthread(struct rt_dlmodule *module,
    rt_thread_t thread) {
    RT_ASSERT(thread->module_id == module);
    (void)module;

    /* lock scheduler to prevent scheduling in cleanup function. */
    rt_enter_critical();
    /* remove thread from thread_list (ready or defunct thread list) */
    rt_list_remove(&(thread->tlist));

    if (RT_THREAD_CLOSE != (thread->stat & RT_THREAD_STAT_MASK) && \
        (RT_Object_Class_Static | RT_Object_Class_Timer) == \
         thread->thread_timer.parent.type) {
        /* release thread timer */
        rt_timer_detach(&(thread->thread_timer));
    }

    /* change stat */
    thread->stat = RT_THREAD_CLOSE;
    /* invoke thread cleanup */
    if (thread->cleanup != RT_NULL)
        thread->cleanup(thread);
    rt_exit_critical();

    #ifdef RT_USING_SIGNALS
        rt_thread_free_sig(thread);
    #endif

    if (thread->type & RT_Object_Class_Static) {
        /* detach object */
        rt_object_detach((rt_object_t)thread);
    #ifdef RT_USING_HEAP
    } else {
        /* release thread's stack */
        RT_KERNEL_FREE(thread->stack_addr);
        /* delete thread object */
        rt_object_delete((rt_object_t)thread);
    #endif
    }
}

rt_err_t dlmodule_destroy(struct rt_dlmodule *module) {
    int i;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* check parameter */
    if (RT_NULL == module) return -RT_ERROR;

    /* can not destroy a running module */
    if (RT_DLMODULE_STAT_RUNNING == module->stat) return -RT_EBUSY;

    /* do module cleanup */
    if (module->cleanup_func) {
        rt_enter_critical();
        module->cleanup_func(module);
        rt_exit_critical();
    }
    LOG_D("mo %s destroy", module->parent.name);

    /* cleanup for all kernel objects inside module*/
    {
        struct rt_object *object = RT_NULL;
        struct rt_list_node *node = RT_NULL;

        /* detach/delete all threads in this module */
        for (node = module->object_list.next;
            node != &(module->object_list); ) {
            int object_type;

            object = rt_list_entry(node, struct rt_object, list);
            object_type = object->type & ~RT_Object_Class_Static;
            /* to next node */
            node = node->next;

            if (object->type & RT_Object_Class_Static) {
                switch (object_type) {
                case RT_Object_Class_Thread:
                    dlmodule_destroy_subthread(module, (rt_thread_t)object);
                    break;
                #ifdef RT_USING_SEMAPHORE
                case RT_Object_Class_Semaphore:
                    rt_sem_detach((rt_sem_t)object);
                    break;
                #endif
                #ifdef RT_USING_MUTEX
                case RT_Object_Class_Mutex:
                    rt_mutex_detach((rt_mutex_t)object);
                    break;
                #endif
                #ifdef RT_USING_EVENT
                case RT_Object_Class_Event:
                    rt_event_detach((rt_event_t)object);
                    break;
                #endif
                #ifdef RT_USING_MAILBOX
                case RT_Object_Class_MailBox:
                    rt_mb_detach((rt_mailbox_t)object);
                    break;
                #endif
                #ifdef RT_USING_MESSAGEQUEUE
                case RT_Object_Class_MessageQueue:
                    rt_mq_detach((rt_mq_t)object);
                    break;
                #endif
                #ifdef RT_USING_MEMHEAP
                case RT_Object_Class_MemHeap:
                    rt_memheap_detach((struct rt_memheap *)object);
                    break;
                #endif
                #ifdef RT_USING_MEMPOOL
                case RT_Object_Class_MemPool:
                    rt_mp_detach((struct rt_mempool *)object);
                    break;
                #endif
                case RT_Object_Class_Timer:
                    rt_timer_detach((rt_timer_t)object);
                    break;
                case RT_Object_Class_Module:
                    dlmodule_destroy((struct rt_dlmodule *)object);
                    break;
                default:
                    LOG_E("bad mo obj type: %x", object_type);
                    break;
                }
            } else { /* if (object->type & RT_Object_Class_Static) */
                switch (object_type) {
                case RT_Object_Class_Thread:
                    dlmodule_destroy_subthread(module, (rt_thread_t)object);
                    break;
                #ifdef RT_USING_SEMAPHORE
                case RT_Object_Class_Semaphore:
                    rt_sem_delete((rt_sem_t)object);
                    break;
                #endif
                #ifdef RT_USING_MUTEX
                case RT_Object_Class_Mutex:
                    rt_mutex_delete((rt_mutex_t)object);
                    break;
                #endif
                #ifdef RT_USING_EVENT
                case RT_Object_Class_Event:
                    rt_event_delete((rt_event_t)object);
                    break;
                #endif
                #ifdef RT_USING_MAILBOX
                case RT_Object_Class_MailBox:
                    rt_mb_delete((rt_mailbox_t)object);
                    break;
                #endif
                #ifdef RT_USING_MESSAGEQUEUE
                case RT_Object_Class_MessageQueue:
                    rt_mq_delete((rt_mq_t)object);
                    break;
                #endif
                #ifdef RT_USING_MEMHEAP
                /* no delete operation */
                #endif
                #ifdef RT_USING_MEMPOOL
                case RT_Object_Class_MemPool:
                    rt_mp_delete((struct rt_mempool*)object);
                    break;
                #endif
                case RT_Object_Class_Timer:
                    rt_timer_delete((rt_timer_t)object);
                    break;
                case RT_Object_Class_Module:
                    dlmodule_destroy((struct rt_dlmodule *)object);
                    break;
                default:
                    LOG_E("bad mo obj type: %x", object_type);
                    break;
                }
            } /* if (object->type & RT_Object_Class_Static) */
        }
    }

    if (module->cmd_line) rt_free(module->cmd_line);
    /* release module symbol table */
    for (i = 0; i < module->nsym; i++) rt_free((void *)module->symtab[i].name);
    if (module->symtab) rt_free(module->symtab);
    /* destory module */
    rt_free(module->mem_space);
    /* delete module object */
    rt_object_delete((rt_object_t)module);

    return RT_EOK;
}

struct rt_dlmodule *dlmodule_self(void) {
    rt_thread_t tid;
    struct rt_dlmodule *ret;

    ret = RT_NULL;
    tid = rt_thread_self();
    if (tid) {
        ret = (struct rt_dlmodule*) tid->module_id;
    }

    return ret;
}

/*
 * Compatible with old API
 */
struct rt_dlmodule *rt_module_self(void) {
    return dlmodule_self();
}

struct rt_dlmodule *dlmodule_load(const char* filename) {
    rt_err_t ret;
    int fd;
    Elf32_Ehdr *elf_hdr;
    struct rt_dlmodule *module;

    ret = RT_EOK;
    fd = -1;
    elf_hdr = RT_NULL;
    module = RT_NULL;

    do {
        fd = open(filename, O_RDONLY, 0);
        if (fd < 0) BREAK_WITH_WARN(RT_EIO, "mo open err");

        /* alloc elf_hdr */
        elf_hdr = (Elf32_Ehdr *)rt_malloc(sizeof(Elf32_Ehdr));
        if (!elf_hdr) BREAK_WITH_WARN(RT_ENOMEM, "elf_hdr mem err");
        LOG_D("elf_hdr: %p", elf_hdr);

        /* load elf_hdr */
        if (sizeof(Elf32_Ehdr) != \
            (rt_uint32_t)read(fd, elf_hdr, sizeof(Elf32_Ehdr)))
            BREAK_WITH_WARN(RT_EIO, "elf_hdr read err");

        /* check elf magic */
        if (rt_memcmp(elf_hdr->e_ident, RTMMAG, SELFMAG) != 0 &&
            rt_memcmp(elf_hdr->e_ident, ELFMAG, SELFMAG) != 0)
            BREAK_WITH_WARN(RT_EIO, "bad elf magic: %s", elf_hdr->e_ident);
        /* check elf class */
        if (elf_hdr->e_ident[EI_CLASS] != ELFCLASS32)
            BREAK_WITH_WARN(RT_EIO, "bad EI_CLASS");
        /* check elf data encoding */
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            if (elf_hdr->e_ident[EI_DATA] != ELFDATA2LSB)
        #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
            if (elf_hdr->e_ident[EI_DATA] != ELFDATA2MSB)
        #endif
                BREAK_WITH_WARN(RT_EIO, "bad EI_DATA");
        /* check elf machine */
        #ifdef __arm__
            if (elf_hdr->e_machine != 0x28)
                BREAK_WITH_WARN(RT_EIO, "bad e_machine");
        #endif

        /* alloc module */
        module = dlmodule_create();
        if (!module) BREAK_WITH_WARN(RT_ENOMEM, "mo mem err");
        /* set the name of module */
        _dlmodule_set_name(module, filename);
        LOG_D("loading %.*s", RT_NAME_MAX, module->parent.name);

        /* load module */
        if (ET_DYN == elf_hdr->e_type) {
            LOG_D("fd %d, DYN %p, hdr %p", fd, module, elf_hdr);
            ret = dlmodule_load_shared_object(fd, elf_hdr, module);
        #ifndef __arm__
        } else if (ET_REL == elf_hdr->e_type) {
            LOG_D("fd %d, REL %p, hdr %p", fd, module, elf_hdr);
            ret = dlmodule_load_relocated_object(fd, elf_hdr, module);
        #endif
        } else {
            BREAK_WITH_WARN(RT_EIO, "bad e_type: %x", elf_hdr->e_type);
        }
        /* check return value */
        if (RT_EOK != ret) break;

        /* release resource */
        close(fd);
        rt_free(elf_hdr);

        /* increase module reference count */
        module->nref++;
        /* deal with cache */
        #ifdef RT_USING_CACHE
            rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, module->mem_space,
                module->mem_size);
            rt_hw_cpu_icache_ops(RT_HW_CACHE_INVALIDATE, module->mem_space,
                module->mem_size);
        #endif
        /* update module initialization and cleanup function */
        if (!module->init_func) {
            module->init_func = dlsym(module, "_init");
        }
        if (!module->init_func) {
            module->init_func = dlsym(module, "module_init");
        }
        if (!module->cleanup_func) {
            module->cleanup_func = dlsym(module, "_fini");
        }
        if (!module->cleanup_func) {
            module->cleanup_func = dlsym(module, "module_cleanup");
        }
        module->stat = RT_DLMODULE_STAT_INIT;

        /* do module initialization */
        if (module->init_func) module->init_func(module);

        return module;
    } while (0);

    if (fd >= 0) close(fd);
    if (elf_hdr) rt_free(elf_hdr);
    if (module) dlmodule_destroy(module);

    return RT_NULL;
}

struct rt_dlmodule *dlmodule_exec(const char* pgname, const char* cmd,
    int cmd_size) {
    struct rt_dlmodule *module = RT_NULL;
    (void)cmd_size;

    module = dlmodule_load(pgname);
    if (!module) return RT_NULL;
    LOG_D("mo @%p", module);

    /* exec module */
    if (module->entry_addr) {
        rt_thread_t tid;

        module->cmd_line = rt_strdup(cmd);
        /* check stack size and priority */
        if (module->priority > RT_THREAD_PRIORITY_MAX) {
            module->priority = RT_THREAD_PRIORITY_MAX - 1;
        }
        if (module->stack_size < MODULE_MIN_STACK_SIZE) {
            module->stack_size = MODULE_MIN_STACK_SIZE;
        } else if (module->stack_size > MODULE_MAX_STACK_SIZE) {
            module->stack_size = MODULE_MAX_STACK_SIZE;
        }
        LOG_D("priority: %d", module->priority);
        LOG_D("stack_size: %d", module->stack_size);

        tid = rt_thread_create(
            module->parent.name,
            _dlmodule_thread_entry, (void*)module, 
            module->stack_size, module->priority,
            MODULE_THREAD_TICK);
        LOG_D("mo tid %x", tid);
        LOG_D("stack_addr: %p", tid->stack_addr);
    
        if (tid) {
            tid->module_id = module;
            module->main_thread = tid;
            rt_thread_startup(tid);
        } else {
            /* destory dl module */
            LOG_E("mo create tsk err");
            dlmodule_destroy(module);
            module = RT_NULL;
        }
    }

    return module;
}

void dlmodule_exit(int ret_code) {
    rt_thread_t thread;
    struct rt_dlmodule *module;

    module = dlmodule_self();
    if (!module) return;

    /* disable scheduling */
    rt_enter_critical();

    /* module is not running */
    if (module->stat != RT_DLMODULE_STAT_RUNNING) {
        /* restore scheduling */
        rt_exit_critical();
        return;
    }

    /* set return code */
    module->ret_code = ret_code;
    LOG_I("mo ret %x", ret_code);

    /* do exit for this module */
    _dlmodule_exit();
    /* the stat of module was changed to CLOSING in _dlmodule_exit */

    thread = module->main_thread;
    if (RT_THREAD_CLOSE == (thread->stat & RT_THREAD_STAT_MASK)) {
        /* main thread already closed */
        rt_exit_critical();
        return;
    }

    /* delete thread: insert to defunct thread list */
    rt_thread_delete(thread);
    /* enable scheduling */
    rt_exit_critical();
}

rt_uint32_t dlmodule_symbol_find(const char *sym_str) {
    /* find in kernel symbol table */
    struct rt_module_symtab *idx;

    for (idx = _rt_module_symtab_begin; idx != _rt_module_symtab_end; idx++) {
        if (rt_strcmp(idx->name, sym_str) == 0) {
            return (rt_uint32_t)idx->addr;
        }
    }

    return 0;
}

int rt_system_dlmodule_init(void) {
    #if defined(__GNUC__) && !defined(__CC_ARM)
    extern int __rtmsymtab_start;
    extern int __rtmsymtab_end;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&__rtmsymtab_start;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&__rtmsymtab_end;
    #elif defined (__CC_ARM)
    extern int RTMSymTab$$Base;
    extern int RTMSymTab$$Limit;

    _rt_module_symtab_begin = (struct rt_module_symtab *)&RTMSymTab$$Base;
    _rt_module_symtab_end   = (struct rt_module_symtab *)&RTMSymTab$$Limit;
    #elif defined (__IAR_SYSTEMS_ICC__)
    _rt_module_symtab_begin = __section_begin("RTMSymTab");
    _rt_module_symtab_end   = __section_end("RTMSymTab");
    #endif

    return 0;
}
INIT_COMPONENT_EXPORT(rt_system_dlmodule_init);

/**
 * This function will find the specified module.
 *
 * @param name the name of module finding
 *
 * @return the module
 */
struct rt_dlmodule *dlmodule_find(const char *name) {
    rt_object_t object;
    struct rt_dlmodule *ret = RT_NULL;

    object = rt_object_find(name, RT_Object_Class_Module);
    if (object) ret = (struct rt_dlmodule *)object;

    return ret;
}
RTM_EXPORT(dlmodule_find);

int list_symbols(int argc, char **argv) {
    rt_uint32_t len;

    if (argc == 1) {
        struct rt_module_symtab *idx;

        /* kernel symbal table */
        rt_kprintf(" [ Kernel Symbals ] \n");
        for (len = 30; len > 0; len--) rt_kprintf("-");
        rt_kprintf("    ----------\n");

        for (idx = _rt_module_symtab_begin;
             idx != _rt_module_symtab_end;
             idx++) {
            rt_kprintf("%-*.s => 0x%08x\n", 30, idx->name, idx->addr);
        }
    } else {
        struct rt_dlmodule *module;
        rt_uint16_t idx;
        int idx2;

        /* module symbal table */
        for (idx2 = 1; idx2 < argc; idx2++) {
            module = dlmodule_find(argv[idx2]);
            if (!module) {
                rt_kprintf(" [ %s ] doesn't load\n", argv[idx2]);
                continue;
            }
            rt_kprintf(" [ %s ] \n", module->parent.name);
            for (len = 30; len > 0; len--) rt_kprintf("-");
            rt_kprintf("    ----------\n");
            for (idx = 0; idx < module->nsym; idx++) {
                rt_kprintf("%-*.s => 0x%08x\n", 30, module->symtab[idx].name,
                    module->symtab[idx].addr);
            }
        }
    }
    rt_kprintf("\n");

    return 0;
}
MSH_CMD_EXPORT(list_symbols, List symbols information);

int list_module(void) {
    struct rt_dlmodule *module;
    struct rt_list_node *list, *node;
    struct rt_object_information *info;
    rt_uint32_t len = RT_NAME_MAX;

    info = rt_object_get_information(RT_Object_Class_Module);
    list = &info->object_list;

    rt_kprintf("%-*.s  ref  address\n", RT_NAME_MAX, "module");
    while (len--) rt_kprintf("-");
    rt_kprintf(     "  ---- --------\n");

    for (node = list->next; node != list; node = node->next) {
        module = (struct rt_dlmodule *)(
            rt_list_entry(node, struct rt_object, list));
        rt_kprintf("%-*.*s  %04d %08x\n", RT_NAME_MAX, RT_NAME_MAX,
                    module->parent.name, module->nref, module->mem_space);
    }
    rt_kprintf("\n");

    return 0;
}
MSH_CMD_EXPORT(list_module, List modules in system);

#endif /* RT_USING_MODULE */
