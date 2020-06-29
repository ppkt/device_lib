add_library(. STATIC
    at24cX.c
    ads1115.c
    bh1750fvi.c
    bme280.c
    ds1307.c
    ds3231.c
    ds_common.c
    hd44780-i2c.c
    ina219.c
    max7219.c
    nrf24l01.c
    ssd1306.c
    w5500.c
    )

target_include_directories(. PUBLIC ..)

target_link_libraries(.
    common_lib
    )

if (${CPU} STREQUAL STM32F0)
    target_link_libraries(. opencm3_stm32f0)
elseif (${CPU} STREQUAL STM32F1)
    target_link_libraries(. opencm3_stm32f1)
endif ()
