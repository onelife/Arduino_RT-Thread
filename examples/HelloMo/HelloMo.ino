/***************************************************************************//**
 * @file    HelloMo.ino
 * @brief   Arduino RT-Thread library "HelloMo" example
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include <rtt.h>

/* ATTENTION
    The following flags in "rtconfig.h" must be turned on:
    - CONFIG_USING_MODULE
 */

/* NOTES
    To build code as "MO" file:
    1. Compile
      - you may copy the compiling command from Arduino's output window (select
      [File-> Preferences-> Show verbose output during: compilation] if you
      can't see the command)
      - add options "-mlong-calls -fPIC"

    2. Link
    - you may copy the link command from Arduino's output window
    - keep only "hello_mo.c.o"
    - remove option "-Wl,--unresolved-symbols=report-all"
    - remove option "-LC:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_362673" 
    - remove option "-TC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\variants\\mkrzero/linker_scripts/gcc/flash_with_bootloader.ld"
    - remove option: "-Wl,--start-group ... -Wl,--end-group"
    - add options "-shared -fPIC -nostdlib -Wl,-marmelf -Wl,-z,max-page-size=0x4"
    - add option to set entry point (e.g. "-Wl,-esay_hello")

    3. Strip
    - use "arm-none-eabi-strip"

    To build code as "SO" file:
    1. Compile
      - same as above

    2. Link
    - same as above, but
    - add option to set entry point to NULL (e.g. "-Wl,-e0")

    3. Strip
    - use "arm-none-eabi-strip"
 */

void setup() {
  RT_T.begin();
  // no code here as RT_T.begin() never return
}

// this function will be called by "Arduino" thread
void loop() {
  // may put some code here that will be run repeatedly
}

/* EXAMPLE

To build "hello_mo.c" as "SO" file:

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -mlong-calls -fPIC -Wall -Wextra -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10809 -DARDUINO_SAMD_MKRZERO -DARDUINO_ARCH_SAMD -DUSE_ARDUINO_MKR_PIN_LAYOUT -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804f -DUSBCON "-DUSB_MANUFACTURER=\"Arduino LLC\"" "-DUSB_PRODUCT=\"Arduino MKRZero\"" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\CMSIS\\4.5.0/CMSIS/Include/" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\CMSIS-Atmel\\1.1.0/CMSIS/Device/ATMEL/" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\cores\\arduino" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\variants\\mkrzero" "-IC:\\Users\\onelife\\Documents\\Arduino\\libraries\\RT-Thread\\src" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\libraries\\SPI" "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\hello_mo.c" -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\hello_mo.c.o"

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-g++" -Os -shared -fPIC -nostdlib -Wl,-e0 -Wl,-marmelf -Wl,-z,max-page-size=0x4 -Wl,--gc-sections -save-temps "-Wl,-Map,C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello_mo.map" --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--warn-common -Wl,--warn-section-align -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello_mo.elf" "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\hello_mo.c.o"

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-strip" -R .hash -R .comment -R .ARM.attributes "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello_mo.elf" -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello.so"


To build "load_mo.c" as "MO" file:

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-gcc" -mcpu=cortex-m0plus -mthumb -c -g -Os -mlong-calls -fPIC -Wall -Wextra -std=gnu11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -MMD -DF_CPU=48000000L -DARDUINO=10809 -DARDUINO_SAMD_MKRZERO -DARDUINO_ARCH_SAMD -DUSE_ARDUINO_MKR_PIN_LAYOUT -D__SAMD21G18A__ -DUSB_VID=0x2341 -DUSB_PID=0x804f -DUSBCON "-DUSB_MANUFACTURER=\"Arduino LLC\"" "-DUSB_PRODUCT=\"Arduino MKRZero\"" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\CMSIS\\4.5.0/CMSIS/Include/" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\CMSIS-Atmel\\1.1.0/CMSIS/Device/ATMEL/" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\cores\\arduino" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\variants\\mkrzero" "-IC:\\Users\\onelife\\Documents\\Arduino\\libraries\\RT-Thread\\src" "-IC:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\hardware\\samd\\1.6.21\\libraries\\SPI" "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\load_mo.c" -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\load_mo.c.o"

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-g++" -Os -shared -fPIC -nostdlib -Wl,-eload_hello -Wl,-marmelf -Wl,-z,max-page-size=0x4 -Wl,--gc-sections -save-temps "-Wl,-Map,C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/load_mo.map" --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--warn-common -Wl,--warn-section-align -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/load_mo.elf" "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\load_mo.c.o"

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-strip" -R .hash -R .comment -R .ARM.attributes "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/load_mo.elf" -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/load.mo"


To build "hello_mo.c" as "MO" file (change a link option and output name):

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-g++" -Os -shared -fPIC -nostdlib -Wl,-esay_hello -Wl,-marmelf -Wl,-z,max-page-size=0x4 -Wl,--gc-sections -save-temps "-Wl,-Map,C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello_mo.map" --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--warn-common -Wl,--warn-section-align -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello_mo.elf" "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658\\sketch\\hello_mo.c.o"

"C:\\Users\\onelife\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\arm-none-eabi-gcc\\4.8.3-2014q1/bin/arm-none-eabi-strip" -R .hash -R .comment -R .ARM.attributes "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello_mo.elf" -o "C:\\Users\\onelife\\AppData\\Local\\Temp\\arduino_build_66658/hello.mo"

 */
