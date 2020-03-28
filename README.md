# swdFirmwareExtractor

The original research of the race condition exploit : https://www.aisec.fraunhofer.de/en/FirmwareProtection.html

This repositry is a fork of @sergachev. It contains a port of the original firmware extractor code to STM32F103C8T6 (bluepill) using HAL drivers.

The firmware is the file RC.bin
Tested successfully on the a STM32F07 target with a bluepill.

PINs  :
PA5 : to be connected to the target power (3.3v) 
PA6 : to be connected to the target reset
PA7 : to be connected to the target SWDIO
PB0 : to be connected to the target SWCLK

PA2 : UART Tx
PA3 : UART Rx
