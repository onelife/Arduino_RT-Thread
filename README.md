# Arduino RT-Thread Library #

This is a fork of [RT-Thread](https://github.com/RT-Thread/rt-thread) project and modified for Arduino.

Currently most of the optional components are removed for the sake of simplicity. Later, some of the those will be brought back.


## Available Components ##

* FinSH (A tiny shell)
  - Support history
  - Support autocompletion
  - Support user defined commands
  - Support user defined variables

* MSH (Module Shell)
  - More in line with Unix shell's usage habits
  - Smaller memory footprint than FinSH
  - Support history
  - Support autocompletion
  - Support user defined commands

* uLog (Micro logging system)
  - Very useful debug tool

* FAT filesystem
  - Support exFAT (off by default)
  - Support long filename (off by default)
  - Support non-English characters (off by default)
  - Support opening multiple files (4 by default)

* SPI SD driver
  - Enabled by default for MKRZero board


## Supported Architectures ##
* SAM (ARM Cortex-M3, Tested with Arduino Due)
* SAMD (ARM Cortex-M0+, Tested with Arduino MKRZero)


## License  ##

| Module     | License |
| ---      | ---       |
| RT-Thread core | Apache License 2.0 |
| FatFS | FatFs (BSD like) License |


## Known Issues ##
None
