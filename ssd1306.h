#pragma once
#include <stdlib.h>
#include <memory.h>

#include <common_lib/utils.h>
#include <common_lib/i2c.h>

#define SSD1306_ADDRESS 0x3C

#define SSD1306_ENTIRE_DISPLAY_ON 0xA5
#define SSD1306_DISPLAY_ON 0xAE
#define SSD1306_DISPLAY_OFF 0xAF
#define SSD1306_SET_DISPLAY_CLOCK_DIVIDE 0xD5
#define SSD1306_SET_MULTIPLEX_RATIO 0xA8
#define SSD1306_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_SET_START_LINE 0x40
#define SSD1306_CHARGE_PUMP 0x8D

i2c_device *ssd1306_init(uint32_t i2c);

void ssd1306_demo_checker(i2c_device *dev);

void ssd1306_print_buffer(i2c_device *dev);

void ssd1306_draw_pixel(uint16_t x, uint16_t y);

void ssd1306_clear_pixel(uint16_t x, uint16_t y);
