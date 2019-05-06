add_library(device_lib STATIC
    at24cX.c
    ds1307.c
    ds3231.c
    ds_common.c
    ina219.c
    max7219.c
    ssd1306.c
    )

target_include_directories(device_lib PUBLIC .)

target_link_libraries(device_lib
    common_lib
    opencm3_stm32f1
    )
