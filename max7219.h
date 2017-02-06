#ifndef __MAX7219_H__
#define __MAX7219_H__

#include <stdlib.h>

#include "common_lib/spi.h"


typedef enum {
    REG_CHANNEL0 = 0x01,
    REG_INTENSITY = 0x0A,
    REG_SCAN_LIMIT = 0x0B,
    REG_SHUTDOWN = 0x0C,
    REG_TEST = 0x0F,
} MAX7219_REGISTER;

typedef struct {
    SPI_TypeDef *spi;
} max7219_device;

max7219_device* max7219_init(SPI_TypeDef *spi);
void max7219_reset(max7219_device* device);
void max7219_self_test(max7219_device *device);
void max7219_clear_display(max7219_device* device);
void max7219_set_brightness(max7219_device* device, uint8_t new_value);
void max7219_turn_all(max7219_device* device);
void max7219_set_led(max7219_device* device, uint8_t row, uint8_t column);
void max7219_reset_led(max7219_device* device, uint8_t row, uint8_t column);
void max7219_set_data(max7219_device* device, uint8_t new_data[8]);

#endif // __MAX7219_H__
