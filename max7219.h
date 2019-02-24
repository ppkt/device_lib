#pragma once

#include <stdlib.h>

#include <common_lib/spi.h>

typedef enum {
    REG_CHANNEL0 = 0x01,
    REG_INTENSITY = 0x0A,
    REG_SCAN_LIMIT = 0x0B,
    REG_SHUTDOWN = 0x0C,
    REG_TEST = 0x0F,
} MAX7219_REGISTER;

/**
 * Initialize module
 */
spi_device *
max7219_init(uint32_t spi);

/**
 * Perform reset operation
 */
void
max7219_reset(const spi_device *device);

/**
 * Turn on and off all LEDs on board
 */
void
max7219_self_test(const spi_device *device);

/**
 * Turn off all LEDs
 */
void
max7219_clear_display(const spi_device *device);

/**
 * Turn on all LEDs
 */
void
max7219_turn_all(const spi_device *device);

/**
 * Set new intensity (brightness) of LEDs
 */
void
max7219_set_brightness(const spi_device *device, uint8_t new_value);

/**
 * Turn on single LED
 */
void
max7219_set_led(const spi_device *device, uint8_t row, uint8_t column);

/**
 * Turn off single LED
 */
void
max7219_reset_led(const spi_device *device, uint8_t row, uint8_t column);

/**
 * Draw whole 8x8 segment
 */
void
max7219_set_data(const spi_device *device, uint8_t new_data[8]);
