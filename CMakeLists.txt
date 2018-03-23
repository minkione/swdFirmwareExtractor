cmake_minimum_required(VERSION 3.2)
project(swdFirmwareExtractor)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")

add_definitions(-DSTM32L152xE -DUSE_HAL_DRIVER)

set(SRC_LIST
        Src/main.c
        Src/stm32l1xx_hal_msp.c
        Src/stm32l1xx_it.c
        Src/system_stm32l1xx.c

        Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal.h
        Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_pwr.h
        Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_rcc.h
        Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_uart.h
)

include_directories(
        Inc
        Drivers/STM32L1xx_HAL_Driver/Inc
        Drivers/STM32L1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32L1xx/Include
        Drivers/CMSIS/Include
)

add_executable(${PROJECT_NAME} ${SRC_LIST})
