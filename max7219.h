#ifndef __MAX7219_H__
#define __MAX7219_H__

#include "common_lib/spi.h"

#define REG_CHANNEL0        0x01
#define REG_INTENSITY       0x0A
#define REG_SCAN_LIMIT      0x0B
#define REG_SHUTDOWN        0x0C
#define REG_TEST            0x0F

void max7219_init(void);
void max7219_reset(void);
void max7219_self_test(void);
void max7219_clear_display(void);
void max7219_set_brightness(uint8_t new_value);
void max7219_turn_all(void);
void max7219_set_led(uint8_t row, uint8_t column);
void max7219_reset_led(uint8_t row, uint8_t column);
void max7219_set_data(uint8_t new_data[8]);

#endif // __MAX7219_H__
