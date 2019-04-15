/***************************************************************************//**
 * @file    context_gcc.S
 * @brief   Arduino RT-Thread library context switching function
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#if defined(ARDUINO_ARCH_SAMD)
#include "cortex-m3/context_gcc.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "cortex-m0/context_gcc.h"
#endif
