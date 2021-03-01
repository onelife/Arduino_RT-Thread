/***************************************************************************//**
 * @file    context_gcc.S
 * @brief   Arduino RT-Thread library context switching function
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#if defined(ARDUINO_ARCH_SAMD)
# include "cortex-m0/context_gcc.h"
#elif defined(ARDUINO_ARCH_SAM)
# include "cortex-m3/context_gcc.h"
#elif defined(STM32F7xx)
# include "cortex-m7/context_gcc.h"
#endif
