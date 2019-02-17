add_library(device_lib STATIC
    ds1307.c
    )

target_include_directories(device_lib PUBLIC .)

target_link_libraries(device_lib
    common_lib
    opencm3_stm32f1
    )
