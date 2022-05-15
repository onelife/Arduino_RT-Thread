/***************************************************************************//**
 * @file    hello_mo.c
 * @brief   Arduino RT-Thread library "HelloMo" example
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include "mo.h"

int say_hello(unsigned char argc, char **argv) {
    char *input;

    if (argc < 2) {
        input = "there";
    } else {
        input = argv[1];
    }
    rt_kprintf("Hello %s!\n\t--- From %s\n", input, argv[0]);

    return 0;
}

void module_init(void *param) {
    struct rt_dlmodule *self = (struct rt_dlmodule *)param;

    rt_kprintf("%s init\n", self->parent.name);
}

void module_cleanup(void *param) {
    struct rt_dlmodule *self = (struct rt_dlmodule *)param;

    rt_kprintf("%s cleanup\n", self->parent.name);
}
