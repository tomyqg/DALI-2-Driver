# DALI-2 Driver
 DALI-2 driver on STM32
This project provides a simple example of a DALI-2 Input Device firmware. It includes a physical layer (dali.c/h), an application layer (dali_application.c/h) and a memory peripheral (dali_memory.c/h). The peripherals are hide in an abstraction layer (tim.c/h, gpio.c/h), making the project more portable between microcontroller and its HAL.