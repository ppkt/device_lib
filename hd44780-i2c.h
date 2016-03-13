#ifndef __HD44780_I2C_H__
#define __HD44780_I2C_H__
#include "stdbool.h"

#include <stm32f10x.h>
#include <stm32f10x_tim.h>

#include "common_lib/utils.h"
#include "common_lib/i2c_dma.h"

#define hd44780_address 0x27

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

void hd44780_init(TIM_TypeDef *timer);
void hd44780_cgram_write(u8 pos, u8 data_[8]);
void hd44780_print(char *string);
void hd44780_go_to_line(u8 line);
void hd44780_go_to(u8 row, u8 col);
void hd44780_cmd(u8 cmd);
void hd44780_char(u8 c);
void hd44780_backlight(bool new_value);


#endif // __HD44780_I2C_H__
