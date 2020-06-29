#pragma once
#include <memory.h>
#include <stdlib.h>

#include <common_lib/gfx.h>
#include <common_lib/i2c.h>
#include <common_lib/utils.h>
#define SSD1306_ADDRESS 0x3C

#define SSD1306_ADDRESSING_MODE 0x20
#define SSD1306_COLUMN_ADDR 0x21
#define SSD1306_PAGE_ADDR 0x22
#define SSD1306_ENTIRE_DISPLAY_ON 0xA5
#define SSD1306_DISPLAY_ON 0xAE
#define SSD1306_DISPLAY_OFF 0xAF
#define SSD1306_SET_DISPLAY_CLOCK_DIVIDE 0xD5
#define SSD1306_SET_MULTIPLEX_RATIO 0xA8
#define SSD1306_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_SET_START_LINE 0x40
#define SSD1306_CHARGE_PUMP 0x8D

typedef struct {
  i2c_device dev;
  uint8_t tx[33];
  uint8_t rx[2];
  uint8_t width;
  uint8_t height;
  uint8_t *buffer;
} ssd1306_device;

error_t ssd1306_init(ssd1306_device *device, uint32_t i2c);

error_t ssd1306_print_buffer(ssd1306_device *device);

error_t ssd1306_partial_print_buffer(ssd1306_device *device,
                                     const gfx_point *p1, const gfx_point *p2);
