# Arduino RT-Thread Library #

This project is forked from [RT-Thread](https://github.com/RT-Thread/rt-thread) (an open source RTOS) and modified for Arduino platform.

I hope this project will release the power of multitasking on Arduino platform.


## Available Drivers ##

| Driver | Dependence | Remark |
| --- | --- | --- |
| SPI | | |
| IIC | | |
| SD Card | SPI | |
| ILI9341 (LCD) | SPI | Tested with Adafruit 2.8" TFT Touch Shield v2 |
| FT6206 (Touch Screen) | IIC | Tested with Adafruit 2.8" TFT Touch Shield v2 |
| SSD1331 (LCD) | SPI | [Tested with TinyCircuits's Pocket Arcade](https://tinycircuits.com/products/pocket-arcade) |
| SSD1306 (LCD) | SPI | Tested with a 0.96" OLED module |


## Available Components ##

| Component | Dependence | Remark |
| --- | --- | --- |
| FinSH | Serial | |
| MSH | FinSH | |
| FAT Filesystem | [ChaN's FatFs](http://elm-chan.org/fsw/ff/00index_e.html) | |
| uLog | Serial | |
| Dynamic Module | | Arduino App |

* FinSH (A tiny shell)
  - Support history
  - Support autocompletion
  - Support user defined commands
  - Support user defined variables

* MSH (Module Shell)
  - More in line with Unix shell's usage habits
  - Smaller memory footprint
  - Support history
  - Support autocompletion
  - Support user defined commands

* FAT Filesystem
  - Support exFAT (off by default)
  - Support long filename (off by default)
  - Support non-English characters (off by default)
  - Support opening multiple files (4 by default)
  - Article: [A Better SD Library with RT-Thread](https://create.arduino.cc/projecthub/onelife/a-better-sd-library-with-rt-thread-242130)

* uLog (Micro logging system)
  - Very useful debug tool

* Dynamic Module (Dynamic Shared Library Linker)
  - Enabled Arduino App
  - Article: [Arduino App with RT-Thread](https://create.arduino.cc/projecthub/onelife/arduino-app-with-rt-thread-96438f)


## Supported Architectures ##

| Architecture | Core | Remark |
| --- | --- | --- |
| SAM | ARM Cortex-M3 | Tested with Arduino Due |
| SAMD | ARM Cortex-M0+ | Tested with Arduino MKRZero |
| GD32V | Bumblebee (RV32IMAC) | Tested with Longan Nano |


## License  ##

| Component | License |
| --- | --- |
| RT-Thread core | Apache License 2.0 |
| FatFS | FatFs (BSD like) License |
