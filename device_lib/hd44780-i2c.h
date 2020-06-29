#pragma once

#include "common_lib/i2c.h"
#include "common_lib/utils.h"

#define HD44780_ADDRESS 0x27

#define HD44780_SCROLL_RIGHT 0x1E
#define HD44780_SCROLL_LEFT 0x18
#define HD44780_MOVE_CURSOR_RIGHT 0x14
#define HD44780_MOVE_CURSOR_LEFT 0x10
#define HD44780_MOVE_TO_HOME 0x02
#define HD44780_CURSOR_UNDERLINED 0x0E
#define HD44780_CURSOR_BLINKED_BLOCK 0x0F
#define HD44780_CURSOR_INVISIBLE 0x0C
#define HD44780_DISPLAY_HIDE 0x08
#define HD44780_DISPLAY_SHOW 0x0C
#define HD44780_DISPLAY_ERASE 0x01

#define HD44780_BACKLIGHT (1u << 3u)
#define HD44780_EN (1u << 2u)
#define HD44780_RW (1u << 1u)
#define HD44780_RS (1u << 0u)

typedef struct {
  i2c_device device;
  uint32_t timer;
} hd44780_device;

error_t hd44780_init(hd44780_device *device, uint32_t i2c, uint8_t address,
                  uint32_t timer);
error_t hd44780_cgram_write(const hd44780_device *device, uint8_t pos,
                            uint8_t data_[8]);
error_t hd44780_print(const hd44780_device *device, const char *string);
error_t hd44780_go_to_line(const hd44780_device *device, uint8_t line);
error_t hd44780_go_to(const hd44780_device *device, uint8_t row, uint8_t col);
error_t hd44780_backlight(const hd44780_device *device, bool new_value);
