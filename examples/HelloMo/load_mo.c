/***************************************************************************//**
 * @file    load_mo.c
 * @brief   Arduino RT-Thread library "HelloMo" example
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include "mo.h"

int load_hello(unsigned char argc, char **argv) {
    struct rt_dlmodule *hello;
    int (*say_hello)(unsigned char argc, char **argv);
    char *param[2];
    (void)argc;
    (void)argv;

    param[0] = "loader";
    param[1] = "there";

    hello = dlopen("hello.so", 0);
    if (!hello) {
        rt_kprintf("Load \"hello.so\" failed\n");
        return -1;
    }
    rt_kprintf("Load \"%s\" ok\n", hello->parent.name);

    say_hello = dlsym(hello, "say_hello");
    if (!say_hello) {
        rt_kprintf("Get symbal \"say_hello\" failed\n");
        return -1;
    }
    rt_kprintf("Symbal \"say_hello\" => %p\n", say_hello);

    say_hello(2, param);

    // while (1) {
    //     rt_thread_sleep(1000);
    // }

    dlclose(hello);

    return 0;
}
