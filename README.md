# Arduino RT-Thread Library #

This is a fork of [RT-Thread](https://github.com/RT-Thread/rt-thread) project and modified for Arduino.

Currently only the kernel APIs are avaiable and most of the optional components are removed for the sake of simplicity. Some of the optional components may be added back later.


## Supported Architectures ##
* SAM (ARM Cortex-M3, Tested with Arduino Due)
* SAMD (ARM Cortex-M0+, Tested with Arduino MKRZero)


## Known Issues ##
* Native USB port is not working currently.

  So for Arduino Due, please connect to host with the "Programming Port"; and for "Arduino MKRZero", please avoid using "Serial" and "rt_kprintf" (by insert `#define CONFIG_NO_CONSOLE` before `#include <rtt.h>`).
